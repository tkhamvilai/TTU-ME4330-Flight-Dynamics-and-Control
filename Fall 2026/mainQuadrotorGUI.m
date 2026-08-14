function mainQuadrotorGUI()
%% Quadcopter simulation with an interactive GUI.
%
% Same vehicle, controller and equations of motion as main.m, driven from a window instead
% of a for loop. Everything that main.m sets once at the top is a live control here: retune
% a gain, move the target, or change the vehicle, and the next time step uses it. Nothing
% needs restarting.
%
% AscTec Pelican, Stevens/Lewis/Johnson Table 8.6-1 p. 654. Imperial units throughout
% (ft, slug, lb, s). Local frame is NED, so positive z is down.
%
% Layout:
%   left  - transport controls, then tabs for Command / Gains / Vehicle, then a readout
%   right - animated 3D trajectory, and strip charts for position, attitude and PWM
%
% The physics live in simStep() at the bottom, which is a copy of the body of the main.m
% loop. If you retune in main.m, the gains here do not follow automatically; the two files
% are independent on purpose so this one can be opened and flown on its own.

frameRate = 30; % display updates per second
histCap = 6000; % samples kept in the trail and strip charts

%% Simulation state shared by every callback
params = defaultParams();
geom = frameGeometry(params);

dt = 0.01; % time step, s
x = zeros(20,1); % state, see main.m for the layout
integ.ol = zeros(3,1); % outerloop integrator, ft-s
integ.il = zeros(3,1); % innerloop integrator, rad-s
simTime = 0;
running = false;
speedFactor = 1;
lastLog = struct('pwm',zeros(4,1),'eul_cmd',zeros(3,1),'F_des',0);

% Waypoint list, held in the units shown in the table: north (ft), east (ft), altitude
% (ft, positive up) and yaw (deg). Altitude is flipped into NED down, and yaw into radians,
% only at the moment the command is handed to the controller.
wp = defaultWaypoints();
wpIndex = 1;
selectedRow = 1;

trail = zeros(3,0); % position history in NED
histT = zeros(1,0);
histPosErr = zeros(1,0);
histEul = zeros(3,0);
histEulCmd = zeros(3,0);
histPwm = zeros(4,0);

gui = struct();

%% Build and launch
buildGui();
resetSim();

simTimer = timer('ExecutionMode','fixedSpacing', ...
                 'Period', round(1/frameRate,3), ...
                 'BusyMode','drop', ...
                 'TimerFcn', @onTick);
gui.fig.CloseRequestFcn = @onClose;
start(simTimer);


%% ------------------------------------------------------------- construction
    function buildGui()
        gui.fig = uifigure('Name','Quadcopter Flight Simulation','Position',[60 60 1340 800]);

        main = uigridlayout(gui.fig,[1 2]);
        main.ColumnWidth = {400,'1x'}; % wide enough for three per-axis gain spinners in a row
        main.RowHeight = {'1x'};

        buildLeft(main);
        buildRight(main);
    end

    function buildLeft(parent)
        panel = uipanel(parent,'Title','Controls');
        lay = uigridlayout(panel,[4 2]);
        lay.RowHeight = {32,34,'1x',110};
        lay.ColumnWidth = {'1x','1x'};

        gui.startBtn = uibutton(lay,'Text','Start','ButtonPushedFcn',@onStartPause);
        gui.startBtn.Layout.Row = 1; gui.startBtn.Layout.Column = 1;

        resetBtn = uibutton(lay,'Text','Reset','ButtonPushedFcn',@onReset);
        resetBtn.Layout.Row = 1; resetBtn.Layout.Column = 2;

        speedLbl = uilabel(lay,'Text','Simulation Speed');
        speedLbl.Layout.Row = 2; speedLbl.Layout.Column = 1;

        gui.speed = uislider(lay,'Limits',[0.25 4],'Value',1, ...
                             'MajorTicks',[0.25 1 2 3 4],'ValueChangedFcn',@(s,~) assignSpeed(s.Value));
        gui.speed.Layout.Row = 2; gui.speed.Layout.Column = 2;

        tabs = uitabgroup(lay);
        tabs.Layout.Row = 3; tabs.Layout.Column = [1 2];
        buildCommandTab(uitab(tabs,'Title','Command'));
        buildGainsTab(uitab(tabs,'Title','Gains'));
        buildVehicleTab(uitab(tabs,'Title','Vehicle'));

        gui.readout = uilabel(lay,'Text',{''},'FontName','Consolas','VerticalAlignment','top');
        gui.readout.Layout.Row = 4; gui.readout.Layout.Column = [1 2];
    end

    function buildCommandTab(tab)
        lay = uigridlayout(tab,[6 2]);
        lay.RowHeight = {'1x',28,30,26,30,30};
        lay.ColumnWidth = {'1.2x','1x'};

        gui.wpTable = uitable(lay, ...
            'ColumnName',{'N (ft)','E (ft)','Alt (ft)','Yaw (deg)'}, ...
            'ColumnEditable',true(1,4), ...
            'Data',wp, ...
            'CellEditCallback',@onWpEdited, ...
            'CellSelectionCallback',@onWpSelected);
        gui.wpTable.Layout.Row = 1; gui.wpTable.Layout.Column = [1 2];

        addBtn = uibutton(lay,'Text','Add','ButtonPushedFcn',@onWpAdd);
        addBtn.Layout.Row = 2; addBtn.Layout.Column = 1;
        removeBtn = uibutton(lay,'Text','Remove','ButtonPushedFcn',@onWpRemove);
        removeBtn.Layout.Row = 2; removeBtn.Layout.Column = 2;

        gui.acceptR = addSpin(lay,3,'Accept radius (ft)',2,[0.25 20],0.25);

        gui.loopChk = uicheckbox(lay,'Text','Repeat the route','Value',false);
        gui.loopChk.Layout.Row = 4; gui.loopChk.Layout.Column = [1 2];

        gui.velMax   = addSpin(lay,5,'Speed limit (ft/s)',5,[0.5 40],0.5);
        gui.attLimit = addSpin(lay,6,'Tilt limit (deg)',45,[5 80],5);
    end

    function buildGainsTab(tab)
        % The outer loop works in north/east/down, so its gains stay per-loop rather than
        % per-axis. The inner loop is where roll, pitch and yaw are genuinely different
        % problems, so each gets its own column.
        lay = uigridlayout(tab,[12 4]);
        lay.RowHeight = {22,26,26,26,26,22,20,26,26,26,26,40};
        lay.ColumnWidth = {'1.25x','1x','1x','1x'};

        outerHead = uilabel(lay,'Text','Outer loop: position to acceleration','FontWeight','bold');
        outerHead.Layout.Row = 1; outerHead.Layout.Column = [1 4];

        gui.KpOl   = addWideSpin(lay,2,'Kp (1/s^2)',1,[0 10],0.1);
        gui.KiOl   = addWideSpin(lay,3,'Ki (1/s^3)',0.1,[0 5],0.05);
        gui.KdOl   = addWideSpin(lay,4,'Kd (1/s)',1,[0.05 10],0.1);
        gui.IlimOl = addWideSpin(lay,5,'I clamp (ft-s)',1,[0 20],0.5);

        innerHead = uilabel(lay,'Text','Inner loop: attitude to moment (ft-lb)','FontWeight','bold');
        innerHead.Layout.Row = 6; innerHead.Layout.Column = [1 4];

        axisNames = {'Roll','Pitch','Yaw'};
        for k = 1:3
            h = uilabel(lay,'Text',axisNames{k},'FontAngle','italic','HorizontalAlignment','center');
            h.Layout.Row = 7; h.Layout.Column = k + 1;
        end

        gui.KpIl   = addAxisSpins(lay,8, 'Kp (per rad)',   0.3, [0 3],    0.02);
        gui.KiIl   = addAxisSpins(lay,9, 'Ki (per rad-s)', 0.1, [0 1.5],  0.02);
        gui.KdIl   = addAxisSpins(lay,10,'Kd (per rad/s)', 0.15,[0.01 1], 0.01);
        gui.IlimIl = addWideSpin(lay,11,'I clamp (rad-s)',1,[0 5],0.25);

        gui.routh = uilabel(lay,'Text',{''},'FontName','Consolas','FontSize',10, ...
                            'VerticalAlignment','top');
        gui.routh.Layout.Row = 12; gui.routh.Layout.Column = [1 4];
    end

    function spins = addAxisSpins(parent,row,label,value,limits,step)
        % One labelled row of three spinners, returned as a 1-by-3 array ordered
        % roll, pitch, yaw so it drops straight into a gain vector.
        lbl = uilabel(parent,'Text',label);
        lbl.Layout.Row = row; lbl.Layout.Column = 1;

        spins = gobjects(1,3);
        for k = 1:3
            spins(k) = uispinner(parent,'Limits',limits,'Value',value,'Step',step, ...
                                 'ValueChangedFcn',@onParamChanged);
            spins(k).Layout.Row = row; spins(k).Layout.Column = k + 1;
        end
    end

    function h = addWideSpin(parent,row,label,value,limits,step)
        % A single spinner stretched across the three axis columns.
        lbl = uilabel(parent,'Text',label);
        lbl.Layout.Row = row; lbl.Layout.Column = 1;

        h = uispinner(parent,'Limits',limits,'Value',value,'Step',step, ...
                      'ValueChangedFcn',@onParamChanged);
        h.Layout.Row = row; h.Layout.Column = [2 4];
    end

    function buildVehicleTab(tab)
        lay = uigridlayout(tab,[7 2]);
        lay.RowHeight = repmat({30},1,7);
        lay.ColumnWidth = {'1.2x','1x'};

        gui.weight = addSpin(lay,1,'Weight (lb)',2.8,[0.5 20],0.1);
        gui.inertia = addSpin(lay,2,'Inertia Jxx (slug-ft^2)',0.032,[0.002 0.5],0.002);
        gui.armLen = addSpin(lay,3,'Arm length (ft)',1,[0.2 10],0.1);
        gui.kt = addSpin(lay,4,'Thrust coeff kt',1.2434e-05,[1e-6 1e-4],1e-6);
        gui.tau = addSpin(lay,5,'Motor tau (s)',0.01,[0.001 0.5],0.005);
        gui.omegaMax = addSpin(lay,6,'Motor omega max (rad/s)',400,[100 1200],25);
        gui.twLabel = uilabel(lay,'Text','','FontName','Consolas','FontSize',10);
        gui.twLabel.Layout.Row = 7; gui.twLabel.Layout.Column = [1 2];
    end

    function h = addSpin(parent,row,label,value,limits,step)
        lbl = uilabel(parent,'Text',label);
        lbl.Layout.Row = row; lbl.Layout.Column = 1;

        h = uispinner(parent,'Limits',limits,'Value',value,'Step',step, ...
                      'ValueChangedFcn',@onParamChanged);
        h.Layout.Row = row; h.Layout.Column = 2;
    end

    function buildRight(parent)
        lay = uigridlayout(parent,[2 1]);
        lay.RowHeight = {'1.6x','1x'};
        lay.ColumnWidth = {'1x'};

        gui.ax3d = uiaxes(lay);
        build3dAxes();

        strip = uigridlayout(lay,[1 3]);
        strip.RowHeight = {'1x'};
        strip.ColumnWidth = {'1x','1x','1x'};

        gui.axPos = uiaxes(strip);
        gui.linePos = plot(gui.axPos,NaN,NaN,'-','Color',[0.15 0.35 0.75],'LineWidth',1.2);
        grid(gui.axPos,'on');
        xlabel(gui.axPos,'time (s)'); ylabel(gui.axPos,'position error (ft)');
        title(gui.axPos,'Distance to target');

        gui.axAtt = uiaxes(strip);
        hold(gui.axAtt,'on');
        gui.lineRoll = plot(gui.axAtt,NaN,NaN,'-r','LineWidth',1.2);
        gui.linePitch = plot(gui.axAtt,NaN,NaN,'-g','LineWidth',1.2);
        gui.lineYaw = plot(gui.axAtt,NaN,NaN,'-b','LineWidth',1.2);
        gui.lineRollC = plot(gui.axAtt,NaN,NaN,'--r');
        gui.linePitchC = plot(gui.axAtt,NaN,NaN,'--g');
        gui.lineYawC = plot(gui.axAtt,NaN,NaN,'--b');
        hold(gui.axAtt,'off');
        grid(gui.axAtt,'on');
        xlabel(gui.axAtt,'time (s)'); ylabel(gui.axAtt,'attitude (deg)');
        title(gui.axAtt,'Attitude, dashed = command');

        gui.axPwm = uiaxes(strip);
        hold(gui.axPwm,'on');
        gui.linePwm = gobjects(1,4);
        for k = 1:4
            gui.linePwm(k) = plot(gui.axPwm,NaN,NaN,'-','LineWidth',1.1);
        end
        gui.pwmLo = yline(gui.axPwm,params.motor.pwmMin,'k--');
        gui.pwmHi = yline(gui.axPwm,params.motor.pwmMax,'k--');
        hold(gui.axPwm,'off');
        grid(gui.axPwm,'on');
        xlabel(gui.axPwm,'time (s)'); ylabel(gui.axPwm,'PWM (\mus)');
        title(gui.axPwm,'Motor commands');
    end

    function build3dAxes()
        ax = gui.ax3d;
        hold(ax,'on'); grid(ax,'on');

        gui.wpLine = plot3(ax,NaN,NaN,NaN,'--o','Color',[0.6 0.6 0.6], ...
                           'MarkerFaceColor',[0.6 0.6 0.6],'MarkerSize',5);
        gui.activeWp = plot3(ax,NaN,NaN,NaN,'o','Color',[0.85 0.33 0.10], ...
                             'MarkerFaceColor',[0.85 0.33 0.10],'MarkerSize',10);
        gui.trailLine = plot3(ax,NaN,NaN,NaN,'-','Color',[0.15 0.35 0.75],'LineWidth',1.2);
        gui.diskLine = plot3(ax,NaN,NaN,NaN,'-','Color',[0.25 0.25 0.25],'LineWidth',1.0);
        gui.rearLine = plot3(ax,NaN,NaN,NaN,'-','Color',[0.25 0.25 0.25],'LineWidth',2.5);
        gui.frontLine = plot3(ax,NaN,NaN,NaN,'-','Color',[0.85 0.33 0.10],'LineWidth',2.5);

        xlabel(ax,'east (ft)'); ylabel(ax,'north (ft)'); zlabel(ax,'altitude (ft)');
        title(ax,'Flight path');
        view(ax,45,22);
        daspect(ax,[1 1 1]);
        hold(ax,'off');
    end


%% ------------------------------------------------------------- simulation
    function resetSim()
        x = zeros(20,1);
        integ.ol = zeros(3,1);
        integ.il = zeros(3,1);
        simTime = 0;
        wpIndex = 1;

        trail = zeros(3,0);
        histT = zeros(1,0);
        histPosErr = zeros(1,0);
        histEul = zeros(3,0);
        histEulCmd = zeros(3,0);
        histPwm = zeros(4,0);

        [~,~,lastLog] = simStep(x,integ,command(),params,dt); % populate the readout at t = 0
        annotateLimits();
        rescale3d();
        updateGraphics();
    end

    function onTick(~,~)
        if ~isvalid(gui.fig) || ~running
            return
        end

        steps = max(1,round(speedFactor/(frameRate*dt)));
        for k = 1:steps
            [x,integ,lastLog] = simStep(x,integ,command(),params,dt);
            simTime = simTime + dt;
            advanceWaypoint(); % checked every step, not every frame, so fast passes still capture
        end

        appendHistory();
        updateGraphics();
        drawnow limitrate
    end

    function cmd = command()
        k = activeIndex();
        cmd.pos_des = [wp(k,1); wp(k,2); -wp(k,3)]; % table altitude is up, NED wants down
        cmd.vel_des = zeros(3,1);
        cmd.eul_des = wrapToPi(deg2rad([0;0;wp(k,4)]));
        cmd.rate_des = zeros(3,1);
    end

    function k = activeIndex()
        k = min(max(wpIndex,1),size(wp,1));
    end

    function P = wpNed(rows)
        % Waypoint positions as a 3-by-N matrix in NED, for drawing and for range checks.
        if nargin < 1
            rows = 1:size(wp,1);
        end
        P = [wp(rows,1)'; wp(rows,2)'; -wp(rows,3)'];
    end

    function advanceWaypoint()
        % Capture the active waypoint once inside its acceptance radius, then move on. The
        % last one is held rather than dropped, so the vehicle parks there instead of
        % drifting once the route is finished.
        k = activeIndex();
        if norm(x(1:3) - wpNed(k)) >= gui.acceptR.Value
            return
        end
        if k < size(wp,1)
            wpIndex = k + 1;
        elseif gui.loopChk.Value
            wpIndex = 1;
        end
    end

    function appendHistory()
        trail(:,end+1) = x(1:3);
        histT(end+1) = simTime;
        histPosErr(end+1) = norm(x(1:3) - command().pos_des);
        histEul(:,end+1) = x(7:9);
        histEulCmd(:,end+1) = lastLog.eul_cmd;
        histPwm(:,end+1) = lastLog.pwm;

        if numel(histT) > histCap
            trail(:,1) = [];
            histT(1) = [];
            histPosErr(1) = [];
            histEul(:,1) = [];
            histEulCmd(:,1) = [];
            histPwm(:,1) = [];
        end
    end


%% ------------------------------------------------------------- graphics
    function updateGraphics()
        R = eul2dcm(x(7:9));
        setBody(gui.frontLine,geom.front,R);
        setBody(gui.rearLine,geom.rear,R);
        setBody(gui.diskLine,geom.disks,R);

        if isempty(trail)
            set(gui.trailLine,'XData',NaN,'YData',NaN,'ZData',NaN);
        else
            p = ned2plot(trail);
            set(gui.trailLine,'XData',p(1,:),'YData',p(2,:),'ZData',p(3,:));
        end

        route = ned2plot(wpNed());
        set(gui.wpLine,'XData',route(1,:),'YData',route(2,:),'ZData',route(3,:));
        act = ned2plot(wpNed(activeIndex()));
        set(gui.activeWp,'XData',act(1),'YData',act(2),'ZData',act(3));

        set(gui.linePos,'XData',histT,'YData',histPosErr);
        set(gui.lineRoll, 'XData',histT,'YData',rad2deg(histEul(1,:)));
        set(gui.linePitch,'XData',histT,'YData',rad2deg(histEul(2,:)));
        set(gui.lineYaw,  'XData',histT,'YData',rad2deg(histEul(3,:)));
        set(gui.lineRollC, 'XData',histT,'YData',rad2deg(histEulCmd(1,:)));
        set(gui.linePitchC,'XData',histT,'YData',rad2deg(histEulCmd(2,:)));
        set(gui.lineYawC,  'XData',histT,'YData',rad2deg(histEulCmd(3,:)));
        for k = 1:4
            set(gui.linePwm(k),'XData',histT,'YData',histPwm(k,:));
        end

        updateReadout();
    end

    function setBody(h,pts,R)
        p = ned2plot(x(1:3) + R*pts);
        set(h,'XData',p(1,:),'YData',p(2,:),'ZData',p(3,:));
    end

    function updateReadout()
        tilt = rad2deg(acos(min(max(cos(x(7))*cos(x(8)),-1),1)));
        err = norm(x(1:3) - command().pos_des);
        gui.readout.Text = { ...
            sprintf('t %7.2f s   waypoint %d of %d', simTime, activeIndex(), size(wp,1)); ...
            sprintf('to go %6.2f ft   accept %5.2f ft', err, gui.acceptR.Value); ...
            sprintf('N %7.2f  E %7.2f  alt %7.2f ft', x(1), x(2), -x(3)); ...
            sprintf('tilt %6.2f deg   yaw %8.2f deg', tilt, rad2deg(x(9))); ...
            sprintf('speed %6.2f ft/s  PWM %6.0f us', norm(x(4:6)), mean(lastLog.pwm)); ...
            sprintf('I inner %6.3f  outer %7.3f', norm(integ.il), norm(integ.ol)) };
    end

    function rescale3d()
        corners = ned2plot([wpNed(), zeros(3,1)]);
        c = (max(corners,[],2) + min(corners,[],2))/2;
        half = max(max(corners,[],2) - min(corners,[],2))/2 + 8;
        gui.ax3d.XLim = c(1) + [-half half];
        gui.ax3d.YLim = c(2) + [-half half];
        gui.ax3d.ZLim = c(3) + [-half half];
    end


%% ------------------------------------------------------------- callbacks
    function onStartPause(~,~)
        running = ~running;
        if running
            gui.startBtn.Text = 'Pause';
        else
            gui.startBtn.Text = 'Start';
        end
    end

    function onReset(~,~)
        running = false;
        gui.startBtn.Text = 'Start';
        resetSim();
    end

    function assignSpeed(v)
        speedFactor = v;
    end

    function onParamChanged(~,~)
        % Rebuild the whole parameter struct so that derived quantities, mass and the mixer
        % above all, can never fall out of step with the values on screen.
        params.body.W = gui.weight.Value;
        params.body.m = params.body.W/params.env.g(3);
        params.body.J = gui.inertia.Value*eye(3);

        params.rotor.l = gui.armLen.Value;
        params.rotor.loc = [ params.rotor.l*sin(pi/4)  params.rotor.l*cos(pi/4) 0;
                            -params.rotor.l*sin(pi/4)  params.rotor.l*cos(pi/4) 0;
                            -params.rotor.l*sin(pi/4) -params.rotor.l*cos(pi/4) 0;
                             params.rotor.l*sin(pi/4) -params.rotor.l*cos(pi/4) 0]';
        params.rotor.kt = gui.kt.Value;
        params.rotor.kq = params.rotor.kt*params.rotor.torque_to_thrust_ratio;
        params.mixer = [-ones(1,params.rotor.n);
                        -params.rotor.loc(2,:);
                         params.rotor.loc(1,:);
                        -params.rotor.dir].*[params.rotor.kt*ones(3,1); params.rotor.kq];

        params.motor.tau = gui.tau.Value;
        params.motor.omegaMax = gui.omegaMax.Value;

        params.ctrl.Kp_ol = gui.KpOl.Value*[1;1;1];
        params.ctrl.Ki_ol = gui.KiOl.Value*[1;1;1];
        params.ctrl.Kd_ol = gui.KdOl.Value*[1;1;1];
        params.ctrl.integral_limit_ol = gui.IlimOl.Value;
        params.ctrl.vel_max = gui.velMax.Value;
        params.ctrl.att_limit = deg2rad(gui.attLimit.Value);

        params.ctrl.Kp_il = [gui.KpIl.Value]'; % roll, pitch, yaw down the column
        params.ctrl.Ki_il = [gui.KiIl.Value]';
        params.ctrl.Kd_il = [gui.KdIl.Value]';
        params.ctrl.integral_limit_il = gui.IlimIl.Value;

        geom = frameGeometry(params);
        annotateLimits();
        rescale3d();
        if ~running
            updateGraphics();
        end
    end

    function annotateLimits()
        % The Routh bound is now per axis, since each one carries its own Kp, Kd and
        % inertia. An axis is unstable on its own account, so the check has to name which.
        inertia = diag(params.body.J);
        KiMax = params.ctrl.Kp_il.*params.ctrl.Kd_il./inertia;
        over = params.ctrl.Ki_il >= KiMax;

        axisNames = {'roll','pitch','yaw'};
        if any(over)
            verdict = sprintf('UNSTABLE: %s', strjoin(axisNames(over(:)'),', '));
        else
            verdict = 'all axes stable';
        end
        gui.routh.Text = { ...
            sprintf('Ki < Kp*Kd/J:  R %.2f  P %.2f  Y %.2f', KiMax(1), KiMax(2), KiMax(3)); ...
            verdict };

        tw = params.rotor.n*params.rotor.kt*params.motor.omegaMax^2/params.body.W;
        gui.twLabel.Text = sprintf('thrust/weight = %.2f   hover PWM = %.0f us', tw, hoverPwm());

        set(gui.pwmLo,'Value',params.motor.pwmMin);
        set(gui.pwmHi,'Value',params.motor.pwmMax);
    end

    function p = hoverPwm()
        omega = sqrt(params.body.W/(params.rotor.n*params.rotor.kt));
        p = params.motor.pwmMin + (params.motor.pwmMax-params.motor.pwmMin)*omega/params.motor.omegaMax;
    end

    function onWpEdited(~,~)
        % A cleared cell arrives as NaN. Fall back to the previous value rather than flying
        % at an undefined point; a cell edit never changes the table size, so the two
        % matrices line up element for element.
        data = gui.wpTable.Data;
        bad = ~isfinite(data);
        data(bad) = wp(bad);
        wp = data;

        gui.wpTable.Data = wp;
        wpIndex = min(max(wpIndex,1),size(wp,1));
        rescale3d();
        updateGraphics();
    end

    function onWpSelected(~,evt)
        if ~isempty(evt.Indices)
            selectedRow = evt.Indices(1,1);
        end
    end

    function onWpAdd(~,~)
        % Insert after the selected row so a route can be extended in the middle, not just
        % at the end. The new point copies its neighbour and lifts 10 ft, as a starting
        % point to edit rather than a guess at what was wanted.
        at = min(max(selectedRow,1),size(wp,1));
        fresh = wp(at,:);
        fresh(3) = fresh(3) + 10;
        wp = [wp(1:at,:); fresh; wp(at+1:end,:)];

        gui.wpTable.Data = wp;
        selectedRow = at + 1;
        rescale3d();
        updateGraphics();
    end

    function onWpRemove(~,~)
        if size(wp,1) <= 1
            return % a route needs at least one point to fly to
        end
        at = min(max(selectedRow,1),size(wp,1));
        wp(at,:) = [];

        gui.wpTable.Data = wp;
        selectedRow = min(selectedRow,size(wp,1));
        wpIndex = min(max(wpIndex,1),size(wp,1));
        rescale3d();
        updateGraphics();
    end

    function onClose(~,~)
        running = false;
        stop(simTimer);
        delete(simTimer);
        delete(gui.fig);
    end

end


%% ------------------------------------------------------------- physics
function [x, integ, log] = simStep(x, integ, cmd, params, dt)
    % One pass of the main.m loop: guidance errors, cascaded PID, mixer, ESC, plant.
    ctrl = params.ctrl;

    % Current states
    pos = x(1:3,1);
    vel = x(4:6,1); % body frame
    eul = x(7:9,1);
    rate = x(10:12,1);

    R = eul2dcm(eul); % body to local
    vel_local = R*vel; % the outerloop works in the local frame
    gB = R'*params.env.g; % gravity in body frame

    % Outerloop errors
    rot_horizon = [cos(eul(3)) sin(eul(3)) 0; -sin(eul(3)) cos(eul(3)) 0; 0 0 1];
    pos_err = rot_horizon*(cmd.pos_des - pos);
    vel_err = rot_horizon*(cmd.vel_des - vel_local);

    % Position Controller
    vp_ol = (ctrl.Kp_ol./ctrl.Kd_ol).*pos_err;
    if norm(vp_ol) > ctrl.vel_max
        vp_ol = vp_ol*ctrl.vel_max/norm(vp_ol);
    end
    vpd_ol = ctrl.Kd_ol.*(vp_ol + vel_err);
    integ.ol = min(max(integ.ol + dt*pos_err, -ctrl.integral_limit_ol),ctrl.integral_limit_ol);
    vpid_ol = vpd_ol + ctrl.Ki_ol.*integ.ol; % outerloop acceleration command

    % Acceleration Command to Desired Attitude
    spf = vpid_ol - gB;
    F_des = spf(3)*params.body.m;

    sdes = vpid_ol;
    sdes(3) = -params.env.g(3) + min(0,sdes(3));
    sdes = -sdes/max(norm(sdes),0.01); % desired body z axis
    yaw_error = wrapToPi(cmd.eul_des(3) - eul(3));
    rot_yaw_err = [ cos(yaw_error) sin(yaw_error) 0;
                   -sin(yaw_error) cos(yaw_error) 0;
                    0 0 1];
    ades = rot_yaw_err*sdes;

    ol_mu = acos(min(max(ades(3),-1),1));
    if ol_mu > ctrl.att_limit
        ades = [sin(ctrl.att_limit)*ades(1:2)/max(norm(ades(1:2)),0.01); cos(ctrl.att_limit)];
    end

    phi_des = -asin(min(max(ades(2),-1),1));
    theta_des = atan2(ades(1),ades(3));
    eul_cmd = [phi_des; theta_des; cmd.eul_des(3)];

    % Innerloop errors
    att_err = eulerRates(eul)\wrapToPi(eul_cmd - eul);
    rate_err = cmd.rate_des - rate;

    % Attitude Controller
    integ.il = min(max(integ.il + dt*att_err, -ctrl.integral_limit_il),ctrl.integral_limit_il);
    M_des = ctrl.Kp_il.*att_err + ctrl.Ki_il.*integ.il + ctrl.Kd_il.*rate_err;

    % Mixer inverse, then the pulse width actually sent to the ESCs
    U_sq = max(pinv(params.mixer)*[F_des; M_des],0);
    omega_des = sqrt(U_sq);
    pwm = params.motor.pwmMin + (params.motor.pwmMax - params.motor.pwmMin)*omega_des/params.motor.omegaMax;
    pwm = min(max(pwm,params.motor.pwmMin),params.motor.pwmMax);

    % Plant
    x_dot = dynamics(x,pwm,params);
    x = x + x_dot*dt;
    x(7:9,1) = wrapToPi(x(7:9,1));

    log.pwm = pwm;
    log.eul_cmd = eul_cmd;
    log.F_des = F_des;
end

function x_dot = dynamics(x,u,params)
    % States. Position and motor phase angle do not affect the derivatives.
    v = x(4:6,1);
    eul = x(7:9,1);
    w = x(10:12,1);
    theta_dot = x(17:20,1);

    % The ESC decodes pulse width into a commanded motor speed
    pwm = min(max(u(1:params.rotor.n,1),params.motor.pwmMin),params.motor.pwmMax);
    omega_cmd = params.motor.omegaMax*(pwm - params.motor.pwmMin)/(params.motor.pwmMax - params.motor.pwmMin);

    m = params.body.m;
    g = params.env.g;
    J = params.body.J;
    tau = params.motor.tau;

    R = eul2dcm(eul);

    FM = params.mixer*(theta_dot.^2);
    F = [0;0;FM(1,1)]; % body, thrust is along -z so this is negative
    M = FM(2:end,1);

    p_dot = R*v;
    v_dot = -cross(w,v) + F/m + R'*g;
    eul_dot = eulerRates(eul)*w;
    w_dot = J\(M-cross(w,J*w));
    theta_dot_dot = 1/tau*(omega_cmd - theta_dot);

    x_dot = [p_dot; v_dot; eul_dot; w_dot; theta_dot; theta_dot_dot];
end

function R = eul2dcm(eul)
    % Body to local (NED) rotation matrix from the 3-2-1 yaw-pitch-roll sequence
    sphi = sin(eul(1)); cphi = cos(eul(1));
    sth  = sin(eul(2)); cth  = cos(eul(2));
    spsi = sin(eul(3)); cpsi = cos(eul(3));

    R = [cth*cpsi  sphi*sth*cpsi-cphi*spsi  cphi*sth*cpsi+sphi*spsi;
         cth*spsi  sphi*sth*spsi+cphi*cpsi  cphi*sth*spsi-sphi*cpsi;
        -sth       sphi*cth                 cphi*cth];
end

function E = eulerRates(eul)
    % Body angular rate to Euler angle rate, eul_dot = E*w. Left divide to go the other way.
    sphi = sin(eul(1)); cphi = cos(eul(1));
    tth  = tan(eul(2)); cth  = cos(eul(2));

    E = [1 sphi*tth cphi*tth;
         0 cphi    -sphi;
         0 sphi/cth cphi/cth];
end


%% ------------------------------------------------------------- setup helpers
function params = defaultParams()
    % Identical to the parameter section of main.m.
    params.env.g = [0; 0; 32.17];
    params.env.rho = 0.002377;

    params.body.W = 2.8;
    params.body.m = params.body.W/params.env.g(3);
    params.body.J = 0.032*eye(3);

    params.rotor.n = 4;
    params.rotor.l = 4;
    params.rotor.loc = [ params.rotor.l*sin(pi/4)  params.rotor.l*cos(pi/4) 0;
                        -params.rotor.l*sin(pi/4)  params.rotor.l*cos(pi/4) 0;
                        -params.rotor.l*sin(pi/4) -params.rotor.l*cos(pi/4) 0;
                         params.rotor.l*sin(pi/4) -params.rotor.l*cos(pi/4) 0]';
    params.rotor.dir = [-1 1 -1 1];
    params.rotor.kt = 1.2434e-05;
    params.rotor.torque_to_thrust_ratio = 0.1;
    params.rotor.kq = params.rotor.kt*params.rotor.torque_to_thrust_ratio;

    params.motor.tau = 0.01;
    params.motor.Jp = 0.00003;
    params.motor.torqueMax = 0.21;
    params.motor.omegaMax = 400;
    params.motor.pwmMin = 1000;
    params.motor.pwmMax = 2000;

    params.mixer = [-ones(1,params.rotor.n);
                    -params.rotor.loc(2,:);
                     params.rotor.loc(1,:);
                    -params.rotor.dir].*[params.rotor.kt*ones(3,1); params.rotor.kq];

    params.ctrl.Kp_ol = [1; 1; 1];
    params.ctrl.Ki_ol = [0.1; 0.1; 0.1];
    params.ctrl.Kd_ol = [1; 1; 1];
    params.ctrl.integral_limit_ol = 1;
    params.ctrl.vel_max = 5;
    params.ctrl.att_limit = deg2rad(45);

    params.ctrl.Kp_il = [0.3; 0.3; 0.3];
    params.ctrl.Ki_il = [0.1; 0.1; 0.1];
    params.ctrl.Kd_il = [0.15; 0.15; 0.15];
    params.ctrl.integral_limit_il = 1.0;
end

function wp = defaultWaypoints()
    % Columns are north (ft), east (ft), altitude (ft, positive up) and yaw (deg). A box
    % with a climb and a turn on each leg, so the yaw and altitude channels are exercised
    % rather than just the horizontal ones.
    wp = [  0    0   20     0;    % climb off the pad
           20    0   20     0;    % run north
           20   20   30    90;    % turn east and climb
            0   20   30   180;    % run south
            0    0   10   -90 ];  % return and descend
end

function geom = frameGeometry(params)
    % Body-frame polylines for drawing the airframe, NaN separated so each group of
    % segments fits in a single line object. Front arms are kept apart from the rear ones
    % so they can be coloured differently, which is the only heading cue in a 3D view.
    loc = params.rotor.loc;
    isFront = loc(1,:) > 0;

    geom.front = armLines(loc(:,isFront));
    geom.rear = armLines(loc(:,~isFront));

    ang = linspace(0,2*pi,25);
    circ = [0.35*params.rotor.l*cos(ang); 0.35*params.rotor.l*sin(ang); zeros(1,numel(ang))];
    n = size(loc,2);
    geom.disks = NaN(3,(numel(ang)+1)*n);
    for k = 1:n
        idx = (numel(ang)+1)*(k-1) + (1:numel(ang));
        geom.disks(:,idx) = loc(:,k) + circ;
    end
end

function lines = armLines(loc)
    n = size(loc,2);
    lines = NaN(3,3*n);
    for k = 1:n
        lines(:,3*k-2) = [0;0;0];
        lines(:,3*k-1) = loc(:,k);
    end
end

function p = ned2plot(v)
    % NED is right for the dynamics but awkward to look at, so plot east, north, altitude.
    p = [v(2,:); v(1,:); -v(3,:)];
end
