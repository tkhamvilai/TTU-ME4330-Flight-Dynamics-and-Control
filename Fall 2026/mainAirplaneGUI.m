function mainAirplaneGUI()
%% Airplane simulation with an interactive GUI.
%
% Same aircraft, autopilot and equations of motion as mainAirplane.m, driven from a window
% instead of a for loop. Everything that script sets once at the top is a live control
% here: switch autopilot mode, retune its gains, move a reference, or change the airframe,
% and the next time step uses it. Nothing needs restarting.
%
% de Havilland DHC-2 Beaver, SI units throughout (m, kg, N, s). Local frame is NED, so
% positive z is down and an altitude of 2202 m is XD = -2202.
%
% Layout:
%   left  - transport controls, then tabs for Autopilot / Pilot & Start / Aircraft
%   right - animated 3D flight path, and strip charts for altitude, airspeed, attitude
%           and control deflection
%
% The Autopilot tab rebuilds itself around whichever mode is selected, so only the
% references and gains that mode actually uses are on screen.
%
% The physics live in simStep() and the functions below it, which are copies of the
% mainAirplane.m loop body and its local functions. The two files are independent on
% purpose so this one can be opened and flown on its own; retuning in mainAirplane.m does
% not follow automatically.

frameRate = 30; % display updates per second
histCap = 20000; % samples kept in the trail and strip charts
minViewHalfSpan = 200; % m, stops the 3D view zooming absurdly close at the start

%% Simulation state shared by every callback
params = defaultParams();

x = zeros(12,1);
integ.theta = 0; % pitch integrator
integ.u = 0; % airspeed integrator
simTime = 0;
running = false;
speedFactor = 1;
lastLog = struct();

trail = zeros(3,0); % position history in NED
histT = zeros(1,0);
histAlt = zeros(1,0);
histV = zeros(1,0);
histEul = zeros(3,0);
histDelta = zeros(4,0);

modeList = {'manual','rateSAS','pitchCAS','altitude','rollYawSAS','bankHeading','velocity','cruise'};
icList = {'doNothing','trimmed','spiral','trimCoupling'};

gui = struct();

%% Build and launch
buildGui();
rebuildAutopilotPanel(); % populates the mode-dependent half of the Autopilot tab
resetSim();

simTimer = timer('ExecutionMode','fixedSpacing', ...
                 'Period', round(1/frameRate,3), ...
                 'BusyMode','drop', ...
                 'TimerFcn', @onTick);
gui.fig.CloseRequestFcn = @onClose;
start(simTimer);


%% ------------------------------------------------------------- construction
    function buildGui()
        gui.fig = uifigure('Name','Airplane Flight Simulation','Position',[50 50 1420 820]);

        main = uigridlayout(gui.fig,[1 2]);
        main.ColumnWidth = {430,'1x'};
        main.RowHeight = {'1x'};

        buildLeft(main);
        buildRight(main);
    end

    function buildLeft(parent)
        panel = uipanel(parent,'Title','Controls');
        lay = uigridlayout(panel,[4 2]);
        lay.RowHeight = {32,34,'1x',118};
        lay.ColumnWidth = {'1x','1x'};

        gui.startBtn = uibutton(lay,'Text','Start','ButtonPushedFcn',@onStartPause);
        gui.startBtn.Layout.Row = 1; gui.startBtn.Layout.Column = 1;

        resetBtn = uibutton(lay,'Text','Reset','ButtonPushedFcn',@onReset);
        resetBtn.Layout.Row = 1; resetBtn.Layout.Column = 2;

        speedLbl = uilabel(lay,'Text','Simulation Speed');
        speedLbl.Layout.Row = 2; speedLbl.Layout.Column = 1;

        % Airplane manoeuvres play out over minutes, so the top speed is well above the
        % quadrotor's; a climb to altitude is unwatchable at 1x.
        gui.speed = uislider(lay,'Limits',[0.25 20],'Value',2, ...
                             'MajorTicks',[0.25 5 10 15 20], ...
                             'ValueChangedFcn',@(s,~) assignSpeed(s.Value));
        gui.speed.Layout.Row = 2; gui.speed.Layout.Column = 2;

        tabs = uitabgroup(lay);
        tabs.Layout.Row = 3; tabs.Layout.Column = [1 2];
        buildAutopilotTab(uitab(tabs,'Title','Autopilot'));
        buildStartTab(uitab(tabs,'Title','Start'));
        buildAirframeTab(uitab(tabs,'Title','Airframe'));
        buildAeroTab(uitab(tabs,'Title','Aero'));

        gui.readout = uilabel(lay,'Text',{''},'FontName','Consolas','VerticalAlignment','top');
        gui.readout.Layout.Row = 4; gui.readout.Layout.Column = [1 2];
    end

    function buildAutopilotTab(tab)
        lay = uigridlayout(tab,[4 2]);
        lay.RowHeight = {30,30,'1x',24};
        lay.ColumnWidth = {'1.1x','1x'};

        modeLbl = uilabel(lay,'Text','Mode');
        modeLbl.Layout.Row = 1; modeLbl.Layout.Column = 1;

        gui.modeDrop = uidropdown(lay,'Items',modeList,'Value',params.ctrl.mode, ...
                                  'ValueChangedFcn',@onModeChanged);
        gui.modeDrop.Layout.Row = 1; gui.modeDrop.Layout.Column = 2;

        gui.iLimTheta = addSpin(lay,2,'Pitch I clamp',params.ctrl.iLimit_theta,[0 10],0.1,@onSharedChanged);

        % Rebuilt whenever the mode changes, so only that mode's knobs are on screen
        gui.apPanel = uipanel(lay,'BorderType','none');
        gui.apPanel.Layout.Row = 3; gui.apPanel.Layout.Column = [1 2];

        gui.modeNote = uilabel(lay,'Text','','FontSize',10,'FontAngle','italic');
        gui.modeNote.Layout.Row = 4; gui.modeNote.Layout.Column = [1 2];
    end

    function buildStartTab(tab)
        lay = uigridlayout(tab,[10 2]);
        lay.RowHeight = repmat({30},1,10);
        lay.ColumnWidth = {'1.1x','1x'};

        head = uilabel(lay,'Text','Pilot trim (feeds the SAS modes)','FontWeight','bold');
        head.Layout.Row = 1; head.Layout.Column = [1 2];

        gui.pAil  = addSpin(lay,2,'Aileron (deg)',0,[-30 30],1,@onPilotChanged);
        gui.pElev = addSpin(lay,3,'Elevator (deg)',0,[-30 30],1,@onPilotChanged);
        gui.pRud  = addSpin(lay,4,'Rudder (deg)',0,[-60 60],1,@onPilotChanged);
        gui.pThr  = addSpin(lay,5,'Throttle (0-1)',0.5,[0 1],0.05,@onPilotChanged);

        head2 = uilabel(lay,'Text','Initial condition (applied on Reset)','FontWeight','bold');
        head2.Layout.Row = 6; head2.Layout.Column = [1 2];

        icLbl = uilabel(lay,'Text','Condition');
        icLbl.Layout.Row = 7; icLbl.Layout.Column = 1;
        gui.icDrop = uidropdown(lay,'Items',icList,'Value','doNothing','ValueChangedFcn',@onReset);
        gui.icDrop.Layout.Row = 7; gui.icDrop.Layout.Column = 2;

        gui.startAlt = addSpin(lay,8,'Start altitude (m)',2202,[100 10000],100,@onReset);
        gui.trimV = addSpin(lay,9,'Trim speed (m/s)',55,[25 120],1,@onReset);

        % The autopilot integrators accumulate once per step rather than per second, so
        % this box retunes every integral loop as well as the integration accuracy.
        gui.dtSpin = addSpin(lay,10,'Time step dt (s)',params.sim.dt,[0.002 0.1],0.002,@onStepChanged);
    end

    function buildAirframeTab(tab)
        % A fixed annotation row on top, then the scrolling spec grid, so the two numbers
        % worth watching stay put while the parameter list scrolls under them.
        outer = uigridlayout(tab,[2 1]);
        outer.RowHeight = {34,'1x'};
        outer.ColumnWidth = {'1x'};
        outer.Padding = [0 0 0 0];
        outer.RowSpacing = 2;

        gui.acLabel = uilabel(outer,'Text',{''},'FontName','Consolas','FontSize',10, ...
                              'VerticalAlignment','top');
        gui.acLabel.Layout.Row = 1;

        holder = uipanel(outer,'BorderType','none');
        holder.Layout.Row = 2;
        addSpecGrid(holder, airframeSpec());
    end

    function buildAeroTab(tab)
        addSpecGrid(tab, aeroSpec());
    end

    function addSpecGrid(parent, spec)
        % Build a labelled spinner per spec row. Rows whose path is empty are section
        % headings. The grid scrolls, which is what makes 30-odd coefficients workable in
        % a 430 px column.
        rows = size(spec,1);
        lay = uigridlayout(parent,[rows 2]);
        lay.RowHeight = repmat({26},1,rows);
        lay.ColumnWidth = {'1.35x','1x'};
        lay.Scrollable = 'on';
        lay.Padding = [4 4 4 4];
        lay.RowSpacing = 3;

        for k = 1:rows
            if isempty(spec{k,2})
                head = uilabel(lay,'Text',spec{k,1},'FontWeight','bold');
                head.Layout.Row = k; head.Layout.Column = [1 2];
                continue
            end

            lbl = uilabel(lay,'Text',spec{k,1});
            lbl.Layout.Row = k; lbl.Layout.Column = 1;

            s = uispinner(lay,'Limits',spec{k,3},'Value',readSpec(spec(k,:)), ...
                          'Step',spec{k,4},'ValueChangedFcn',@onSpecChanged);
            s.UserData = spec(k,:);
            s.Layout.Row = k; s.Layout.Column = 2;
        end
    end

    function h = addSpin(parent,row,label,value,limits,step,callback)
        lbl = uilabel(parent,'Text',label);
        lbl.Layout.Row = row; lbl.Layout.Column = 1;

        h = uispinner(parent,'Limits',limits,'Value',value,'Step',step,'ValueChangedFcn',callback);
        h.Layout.Row = row; h.Layout.Column = 2;
    end

    function buildRight(parent)
        lay = uigridlayout(parent,[2 1]);
        lay.RowHeight = {'1.5x','1x'};
        lay.ColumnWidth = {'1x'};

        gui.ax3d = uiaxes(lay);
        build3dAxes();

        strip = uigridlayout(lay,[1 4]);
        strip.RowHeight = {'1x'};
        strip.ColumnWidth = {'1x','1x','1x','1x'};

        gui.axAlt = uiaxes(strip);
        gui.lineAlt = plot(gui.axAlt,NaN,NaN,'-','Color',[0.15 0.35 0.75],'LineWidth',1.2);
        gui.refAlt = yline(gui.axAlt,0,'--','Color',[0.85 0.33 0.10],'Visible','off');
        grid(gui.axAlt,'on');
        xlabel(gui.axAlt,'time (s)'); ylabel(gui.axAlt,'altitude (m)');
        title(gui.axAlt,'Altitude');

        gui.axV = uiaxes(strip);
        gui.lineV = plot(gui.axV,NaN,NaN,'-','Color',[0.15 0.35 0.75],'LineWidth',1.2);
        gui.refV = yline(gui.axV,0,'--','Color',[0.85 0.33 0.10],'Visible','off');
        grid(gui.axV,'on');
        xlabel(gui.axV,'time (s)'); ylabel(gui.axV,'airspeed (m/s)');
        title(gui.axV,'Airspeed');

        gui.axAtt = uiaxes(strip);
        hold(gui.axAtt,'on');
        gui.lineRoll  = plot(gui.axAtt,NaN,NaN,'-r','LineWidth',1.1);
        gui.linePitch = plot(gui.axAtt,NaN,NaN,'-g','LineWidth',1.1);
        gui.lineYaw   = plot(gui.axAtt,NaN,NaN,'-b','LineWidth',1.1);
        hold(gui.axAtt,'off');
        grid(gui.axAtt,'on');
        xlabel(gui.axAtt,'time (s)'); ylabel(gui.axAtt,'attitude (deg)');
        title(gui.axAtt,'\phi red, \theta green, \psi blue');

        gui.axCtrl = uiaxes(strip);
        hold(gui.axCtrl,'on');
        gui.lineAil  = plot(gui.axCtrl,NaN,NaN,'-r','LineWidth',1.1);
        gui.lineElev = plot(gui.axCtrl,NaN,NaN,'-g','LineWidth',1.1);
        gui.lineRud  = plot(gui.axCtrl,NaN,NaN,'-b','LineWidth',1.1);
        hold(gui.axCtrl,'off');
        grid(gui.axCtrl,'on');
        xlabel(gui.axCtrl,'time (s)'); ylabel(gui.axCtrl,'deflection (deg)');
        title(gui.axCtrl,'ail red, elev green, rud blue');
    end

    function build3dAxes()
        ax = gui.ax3d;
        hold(ax,'on'); grid(ax,'on');

        gui.trailLine = plot3(ax,NaN,NaN,NaN,'-','Color',[0.15 0.35 0.75],'LineWidth',1.2);
        gui.tailLine = plot3(ax,NaN,NaN,NaN,'-','Color',[0.25 0.25 0.25],'LineWidth',2);
        gui.frameLine = plot3(ax,NaN,NaN,NaN,'-','Color',[0.85 0.33 0.10],'LineWidth',2.5);

        xlabel(ax,'east (m)'); ylabel(ax,'north (m)'); zlabel(ax,'altitude (m)');
        title(ax,'Flight path');
        view(ax,45,20);
        daspect(ax,[1 1 1]);
        hold(ax,'off');
    end

    function rebuildAutopilotPanel()
        % Tear the old controls down and lay out the ones the active mode actually uses.
        delete(gui.apPanel.Children);

        spec = modeSpec(params.ctrl.mode);
        rows = max(size(spec,1),1);

        lay = uigridlayout(gui.apPanel,[rows 2]);
        lay.RowHeight = repmat({28},1,rows);
        lay.ColumnWidth = {'1.15x','1x'};
        lay.Padding = [0 0 0 0];

        for k = 1:size(spec,1)
            lbl = uilabel(lay,'Text',spec{k,1});
            lbl.Layout.Row = k; lbl.Layout.Column = 1;

            stored = params.ctrl.(params.ctrl.mode).(spec{k,2});
            s = uispinner(lay,'Limits',spec{k,3},'Value',stored*spec{k,5}, ...
                          'Step',spec{k,4},'ValueChangedFcn',@onModeGainChanged);
            s.UserData = struct('field',spec{k,2},'scale',spec{k,5});
            s.Layout.Row = k; s.Layout.Column = 2;
        end

        gui.modeNote.Text = modeNote(params.ctrl.mode);
        refreshReferenceLines();
    end


%% ------------------------------------------------------------- simulation
    function resetSim()
        % initialState reads params.pilot for the 'doNothing' case, so the pilot trim on
        % screen has to be in params before the initial state is built.
        readPilot();
        alt = gui.startAlt.Value;
        [x, params.pilot] = initialState(gui.icDrop.Value, alt, gui.trimV.Value, params);
        writePilot(); % a solved trim overwrites the pilot boxes, so push it back to screen

        integ.theta = 0;
        integ.u = 0;
        simTime = 0;

        trail = zeros(3,0);
        histT = zeros(1,0);
        histAlt = zeros(1,0);
        histV = zeros(1,0);
        histEul = zeros(3,0);
        histDelta = zeros(4,0);

        [~,~,lastLog] = simStep(x,integ,params,params.sim.dt); % populate the readout at t = 0
        annotateAircraft();
        rescale3d();
        updateGraphics();
    end

    function onTick(~,~)
        if ~isvalid(gui.fig) || ~running
            return
        end

        dtStep = params.sim.dt;
        steps = max(1,round(speedFactor/(frameRate*dtStep)));
        for k = 1:steps
            [x,integ,lastLog] = simStep(x,integ,params,dtStep);
            simTime = simTime + dtStep;
            if ~all(isfinite(x))
                running = false;
                gui.startBtn.Text = 'Start';
                gui.modeNote.Text = 'diverged: state went non-finite, press Reset';
                return
            end
        end

        appendHistory();
        updateGraphics();
        drawnow limitrate
    end

    function appendHistory()
        trail(:,end+1) = x(1:3);
        histT(end+1) = simTime;
        histAlt(end+1) = -x(3);
        histV(end+1) = lastLog.air.V;
        histEul(:,end+1) = x(7:9);
        histDelta(:,end+1) = [lastLog.delta.aileron; lastLog.delta.elevator; ...
                              lastLog.delta.rudder; lastLog.delta.throttle];

        if numel(histT) > histCap
            trail(:,1) = [];
            histT(1) = [];
            histAlt(1) = [];
            histV(1) = [];
            histEul(:,1) = [];
            histDelta(:,1) = [];
        end
    end


%% ------------------------------------------------------------- graphics
    function updateGraphics()
        rescale3d(); % first, so the glyph is sized against the view it will be drawn in

        R = eul2dcm(x(7:9));
        geom = airframeGeometry(params, viewScale());
        setBody(gui.frameLine, geom.frame, R);
        setBody(gui.tailLine, geom.tail, R);

        if isempty(trail)
            set(gui.trailLine,'XData',NaN,'YData',NaN,'ZData',NaN);
        else
            p = ned2plot(trail);
            set(gui.trailLine,'XData',p(1,:),'YData',p(2,:),'ZData',p(3,:));
        end

        set(gui.lineAlt,'XData',histT,'YData',histAlt);
        set(gui.lineV,'XData',histT,'YData',histV);
        set(gui.lineRoll, 'XData',histT,'YData',rad2deg(histEul(1,:)));
        set(gui.linePitch,'XData',histT,'YData',rad2deg(histEul(2,:)));
        set(gui.lineYaw,  'XData',histT,'YData',rad2deg(histEul(3,:)));
        set(gui.lineAil, 'XData',histT,'YData',rad2deg(histDelta(1,:)));
        set(gui.lineElev,'XData',histT,'YData',rad2deg(histDelta(2,:)));
        set(gui.lineRud, 'XData',histT,'YData',rad2deg(histDelta(3,:)));

        updateReadout();
    end

    function setBody(h,pts,R)
        p = ned2plot(x(1:3) + R*pts);
        set(h,'XData',p(1,:),'YData',p(2,:),'ZData',p(3,:));
    end

    function s = viewScale()
        % The aircraft is metres long and the flight path is kilometres, so a to-scale
        % glyph would be a single pixel. Scale it to a fixed fraction of the current view
        % instead, the same trick trajectory_plot.m uses.
        s = max(diff(gui.ax3d.XLim), 1)/(12*params.geom.b);
    end

    function rescale3d()
        pts = ned2plot([trail, x(1:3)]);
        centre = (max(pts,[],2) + min(pts,[],2))/2;
        half = max([max(pts,[],2) - min(pts,[],2); 0])/2 + minViewHalfSpan;

        gui.ax3d.XLim = centre(1) + [-half half];
        gui.ax3d.YLim = centre(2) + [-half half];
        gui.ax3d.ZLim = centre(3) + [-half half];
    end

    function updateReadout()
        a = lastLog.air;
        d = lastLog.delta;
        gui.readout.Text = { ...
            sprintf('t %7.1f s   mode %s', simTime, params.ctrl.mode); ...
            sprintf('alt %8.1f m   V %7.2f m/s', -x(3), a.V); ...
            sprintf('alpha %5.2f  beta %5.2f deg', rad2deg(a.alpha), rad2deg(a.beta)); ...
            sprintf('phi %6.2f  th %6.2f  psi %7.2f', rad2deg(x(7)), rad2deg(x(8)), rad2deg(x(9))); ...
            sprintf('ail %5.1f  ele %5.1f  rud %5.1f deg', rad2deg(d.aileron), rad2deg(d.elevator), rad2deg(d.rudder)); ...
            sprintf('throttle %4.2f   I: th %5.2f  u %6.1f', d.throttle, integ.theta, integ.u) };
    end

    function refreshReferenceLines()
        % Show the altitude and speed targets only for the modes that actually hold them.
        [hRef, vRef] = modeReferences(params);
        setRef(gui.refAlt, hRef);
        setRef(gui.refV, vRef);
    end

    function setRef(h, value)
        if isnan(value)
            h.Visible = 'off';
        else
            h.Value = value;
            h.Visible = 'on';
        end
    end

    function annotateAircraft()
        % Two numbers worth watching while the airframe knobs are being turned.
        wingLoading = params.body.m*params.env.g(3)/params.geom.S;
        if params.aero.Cm_a < 0
            verdict = 'statically stable';
        else
            verdict = 'STATICALLY UNSTABLE';
        end
        gui.acLabel.Text = { ...
            sprintf('W/S = %.0f N/m^2', wingLoading); ...
            sprintf('Cm_alpha %+.2f  (%s)', params.aero.Cm_a, verdict) };
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

    function onModeChanged(src,~)
        params.ctrl.mode = src.Value;
        % Integrators carry the trim of the loop that owned them, so hand a fresh mode a
        % clean slate rather than someone else's wind-up.
        integ.theta = 0;
        integ.u = 0;
        rebuildAutopilotPanel();
        if ~running
            updateGraphics();
        end
    end

    function onModeGainChanged(src,~)
        d = src.UserData;
        params.ctrl.(params.ctrl.mode).(d.field) = src.Value/d.scale;
        refreshReferenceLines();
    end

    function onSharedChanged(~,~)
        params.ctrl.iLimit_theta = gui.iLimTheta.Value;
    end

    function onPilotChanged(~,~)
        readPilot();
    end

    function readPilot()
        params.pilot.aileron  = deg2rad(gui.pAil.Value);
        params.pilot.elevator = deg2rad(gui.pElev.Value);
        params.pilot.rudder   = deg2rad(gui.pRud.Value);
        params.pilot.throttle = gui.pThr.Value;
    end

    function writePilot()
        gui.pAil.Value  = clampToLimits(rad2deg(params.pilot.aileron), gui.pAil.Limits);
        gui.pElev.Value = clampToLimits(rad2deg(params.pilot.elevator), gui.pElev.Limits);
        gui.pRud.Value  = clampToLimits(rad2deg(params.pilot.rudder), gui.pRud.Limits);
        gui.pThr.Value  = clampToLimits(params.pilot.throttle, gui.pThr.Limits);
    end

    function onSpecChanged(src,~)
        % One callback for every airframe and aerodynamic box. The spec row carried on the
        % spinner says where the value lives, so nothing here needs to know which knob it was.
        row = src.UserData;
        writeSpec(row, src.Value);

        % The propeller table is generated from its amplitude, so regenerate it rather than
        % leaving the curve and the number that defines it disagreeing.
        params.prop.thrust = params.prop.scale*(1 + tanh(-3:3));

        annotateAircraft();
        if ~running
            updateGraphics();
        end
    end

    function value = readSpec(row)
        % Value for a spec row, in display units.
        stored = getPath(params, row{2});
        if ~isempty(row{6})
            idx = row{6}{1};
            stored = stored(idx(1), idx(2));
        end
        value = stored*row{5};
    end

    function writeSpec(row, value)
        stored = value/row{5};
        if isempty(row{6})
            params = setPath(params, row{2}, stored);
            return
        end

        % Matrix entry. Several indices means a symmetric pair such as Ixz, which has to be
        % written to both sides or the inertia matrix stops being symmetric.
        target = getPath(params, row{2});
        for k = 1:numel(row{6})
            idx = row{6}{k};
            target(idx(1), idx(2)) = stored;
        end
        params = setPath(params, row{2}, target);
    end

    function onStepChanged(~,~)
        params.sim.dt = gui.dtSpin.Value;
    end

    function onClose(~,~)
        running = false;
        stop(simTimer);
        delete(simTimer);
        delete(gui.fig);
    end

end


%% ------------------------------------------------------------- autopilot panel spec
function spec = modeSpec(mode)
% One row per knob: {label, field in params.ctrl.<mode>, display limits, step, scale}.
% scale converts the stored value to what is shown, so an angle held in radians is
% presented in degrees.
deg = 180/pi;
switch mode
    case 'manual'
        spec = cell(0,5);

    case 'rateSAS'
        spec = { 'Kp  roll rate',  'Kp', [0 20], 0.5, 1;
                 'Kq  pitch rate', 'Kq', [0 20], 0.5, 1;
                 'Kr  yaw rate',   'Kr', [0 20], 0.5, 1 };

    case 'pitchCAS'
        spec = { 'Pitch ref (deg)', 'theta_ref', [-30 30], 1,    deg;
                 'Throttle',        'throttle',  [0 1],    0.05, 1;
                 'Kp',              'Kp',        [0 10],   0.1,  1;
                 'Ki',              'Ki',        [0 2],    0.01, 1;
                 'Kd',              'Kd',        [0 10],   0.1,  1 };

    case 'altitude'
        spec = { 'Altitude ref (m)', 'h_ref',    [0 10000], 100,  1;
                 'Kp_h alt to pitch','Kp_h',     [0 0.2],   0.005, 1;
                 'Kd_h climb damp',  'Kd_h',     [0 0.2],   0.005, 1;
                 'Pitch limit (deg)','thetaMax', [1 45],    1,    deg;
                 'Kp',               'Kp',       [0 10],    0.1,  1;
                 'Ki',               'Ki',       [0 2],     0.01, 1;
                 'Kd',               'Kd',       [0 10],    0.1,  1;
                 'Kt alt to thrust', 'Kt',       [0 5],     0.05, 1 };

    case 'rollYawSAS'
        spec = { 'Bank ref (deg)',    'phi_ref', [-60 60], 1,   deg;
                 'Yaw rate ref (d/s)','r_ref',   [-20 20], 0.5, deg;
                 'Kpp bank',          'Kpp',     [0 10],   0.1, 1;
                 'Kpd roll rate',     'Kpd',     [0 10],   0.1, 1;
                 'Kr  yaw rate',      'Kr',      [0 10],   0.1, 1 };

    case 'bankHeading'
        spec = { 'Bank ref (deg)',    'phi_ref', [-60 60],   1, deg;
                 'Heading ref (deg)', 'psi_ref', [-180 180], 5, deg };

    case 'velocity'
        spec = { 'Speed ref (m/s)', 'u_ref',     [20 120], 1,    1;
                 'Pitch ref (deg)', 'theta_ref', [-30 30], 1,    deg;
                 'Kp_u',            'Kp_u',      [0 20],   0.5,  1;
                 'Ki_u',            'Ki_u',      [0 20],   0.5,  1;
                 'Kp',              'Kp',        [0 10],   0.1,  1;
                 'Ki',              'Ki',        [0 2],    0.01, 1;
                 'Kd',              'Kd',        [0 10],   0.1,  1 };

    case 'cruise'
        spec = { 'Altitude ref (m)', 'h_ref',    [0 10000], 100,   1;
                 'Speed ref (m/s)',  'u_ref',    [20 120],  1,     1;
                 'Kp_u',             'Kp_u',     [0 20],    0.5,   1;
                 'Ki_u',             'Ki_u',     [0 20],    0.5,   1;
                 'Kp_h alt to pitch','Kp_h',     [0 0.2],   0.005, 1;
                 'Kd_h climb damp',  'Kd_h',     [0 0.2],   0.005, 1;
                 'Pitch limit (deg)','thetaMax', [1 45],    1,     deg;
                 'Kp',               'Kp',       [0 10],    0.1,   1;
                 'Ki',               'Ki',       [0 2],     0.01,  1;
                 'Kd',               'Kd',       [0 10],    0.1,   1 };

    otherwise
        error('mainAirplaneGUI:mode','unknown autopilot mode "%s"', mode);
end
end

function note = modeNote(mode)
% The one-line description of each example, from flightController.m.
switch mode
    case 'manual',      note = 'pilot trim straight through, no feedback';
    case 'rateSAS',     note = 'rate stability augmentation about trim';
    case 'pitchCAS',    note = 'climb holding a constant pitch angle';
    case 'altitude',    note = 'climb to an altitude and level out';
    case 'rollYawSAS',  note = 'turn coordinator / emergency descent';
    case 'bankHeading', note = 'LQR bank and heading hold, forward-slip landing';
    case 'velocity',    note = 'velocity hold, speed control and gliding';
    case 'cruise',      note = 'altitude hold and velocity hold together';
    otherwise,          note = '';
end
end

function [hRef, vRef] = modeReferences(params)
% Altitude and speed targets for the strip charts, NaN where the mode holds neither.
hRef = NaN;
vRef = NaN;
switch params.ctrl.mode
    case 'altitude'
        hRef = params.ctrl.altitude.h_ref;
    case 'velocity'
        vRef = params.ctrl.velocity.u_ref;
    case 'cruise'
        hRef = params.ctrl.cruise.h_ref;
        vRef = params.ctrl.cruise.u_ref;
end
end

function spec = airframeSpec()
% Every constant that describes the vehicle and its environment.
% Columns: {label, dotted path in params, display limits, step, scale, matrix indices}.
% An empty path makes the row a section heading. scale converts stored to displayed, so an
% angle held in radians is shown in degrees. The index column is empty for plain fields and
% carries one or more [row col] pairs for entries of a matrix.
deg = 180/pi;
spec = {
    'Environment',           '',                 [],            [],     1,   {};
    'Gravity (m/s^2)',       'env.g',            [1 30],        0.1,    1,   {[3 1]};
    'Sea level density',     'env.rho0',         [0.1 2],       0.005,  1,   {};
    'Sea level temp (K)',    'env.T0',           [200 350],     1,      1,   {};
    'Lapse rate (K/m)',      'env.lapse',        [0 0.02],      0.0005, 1,   {};
    'Gas constant (J/kg-K)', 'env.R',            [200 400],     1,      1,   {};

    'Mass and inertia',      '',                 [],            [],     1,   {};
    'Mass (kg)',             'body.m',           [200 20000],   50,     1,   {};
    'Ixx (kg-m^2)',          'body.J',           [100 100000],  100,    1,   {[1 1]};
    'Iyy (kg-m^2)',          'body.J',           [100 100000],  100,    1,   {[2 2]};
    'Izz (kg-m^2)',          'body.J',           [100 100000],  100,    1,   {[3 3]};
    'Ixz (kg-m^2)',          'body.J',           [-5000 5000],  10,     1,   {[1 3],[3 1]};

    'Reference geometry',    '',                 [],            [],     1,   {};
    'Wing area S (m^2)',     'geom.S',           [1 200],       0.5,    1,   {};
    'Span b (m)',            'geom.b',           [1 80],        0.5,    1,   {};
    'Chord c (m)',           'geom.c',           [0.1 10],      0.05,   1,   {};

    'Propulsion',            '',                 [],            [],     1,   {};
    'Thrust curve scale (N)','prop.scale',       [100 50000],   250,    1,   {};

    'Actuator limits',       '',                 [],            [],     1,   {};
    'Aileron max (deg)',     'act.aileronMax',   [1 60],        1,      deg, {};
    'Elevator max (deg)',    'act.elevatorMax',  [1 60],        1,      deg, {};
    'Rudder max (deg)',      'act.rudderMax',    [1 90],        1,      deg, {};
    };
end

function spec = aeroSpec()
% The full body-axis coefficient set, same grouping as flightSim_createAircraft.m.
spec = {
    'Longitudinal',   '',              [],          [],     1, {};
    'CX_0',           'aero.CX_0',     [-1 1],      0.005,  1, {};
    'CX_alpha',       'aero.CX_a',     [-5 5],      0.01,   1, {};
    'CX_qcV',         'aero.CX_qcV',   [-20 20],    0.05,   1, {};
    'CZ_0',           'aero.CZ_0',     [-2 2],      0.005,  1, {};
    'CZ_alpha',       'aero.CZ_a',     [-15 0],     0.05,   1, {};
    'CZ_qcV',         'aero.CZ_qcV',   [-30 5],     0.05,   1, {};
    'Cm_0',           'aero.Cm_0',     [-1 1],      0.005,  1, {};
    'Cm_alpha',       'aero.Cm_a',     [-5 2],      0.02,   1, {};
    'Cm_qcV',         'aero.Cm_qcV',   [-40 5],     0.5,    1, {};

    'Lateral and directional', '',     [],          [],     1, {};
    'CY_beta',        'aero.CY_b',     [-3 1],      0.02,   1, {};
    'CY_pb2V',        'aero.CY_pb2V',  [-3 3],      0.02,   1, {};
    'CY_rb2V',        'aero.CY_rb2V',  [-3 3],      0.02,   1, {};
    'Cl_beta',        'aero.Cl_b',     [-1 1],      0.005,  1, {};
    'Cl_pb2V',        'aero.Cl_pb2V',  [-3 1],      0.01,   1, {};
    'Cl_rb2V',        'aero.Cl_rb2V',  [-1 2],      0.01,   1, {};
    'Cm_rb2V',        'aero.Cm_rb2V',  [-3 3],      0.02,   1, {};
    'Cn_0',           'aero.Cn_0',     [-0.5 0.5],  0.002,  1, {};
    'Cn_beta',        'aero.Cn_b',     [-0.5 0.5],  0.002,  1, {};
    'Cn_pb2V',        'aero.Cn_pb2V',  [-2 2],      0.01,   1, {};
    'Cn_rb2V',        'aero.Cn_rb2V',  [-2 2],      0.01,   1, {};

    'Control derivatives', '',         [],          [],     1, {};
    'CZ_elevator',    'aero.CZ_de',    [-5 5],      0.02,   1, {};
    'Cm_elevator',    'aero.Cm_de',    [-10 5],     0.05,   1, {};
    'Cl_aileron',     'aero.Cl_da',    [-1 1],      0.005,  1, {};
    'Cn_aileron',     'aero.Cn_da',    [-1 1],      0.002,  1, {};
    'CY_aileron',     'aero.CY_da',    [-1 1],      0.005,  1, {};
    'Cl_rudder',      'aero.Cl_dr',    [-1 1],      0.002,  1, {};
    'Cn_rudder',      'aero.Cn_dr',    [-1 1],      0.005,  1, {};
    'CY_rudder',      'aero.CY_dr',    [-1 1],      0.005,  1, {};
    };
end

function value = getPath(s, path)
% Read a nested field named by a dotted path, so the spec tables can address any parameter.
fields = strsplit(path,'.');
value = s;
for k = 1:numel(fields)
    value = value.(fields{k});
end
end

function s = setPath(s, path, value)
% Write a nested field named by a dotted path.
fields = strsplit(path,'.');
if isscalar(fields)
    s.(fields{1}) = value;
else
    s.(fields{1}) = setPath(s.(fields{1}), strjoin(fields(2:end),'.'), value);
end
end

function geom = airframeGeometry(params, scale)
% A simple planform glyph in the body frame, NaN separated. Dimensions are fractions of
% the span so the shape survives a change of aircraft size.
b = params.geom.b*scale;

geom.frame = [ 0.45*b -0.55*b  NaN   0.05*b  0.05*b;   % fuselage, then wing
                    0       0  NaN  -0.50*b  0.50*b;
                    0       0  NaN        0       0 ];

geom.tail = [ -0.45*b -0.45*b  NaN  -0.45*b -0.45*b;   % tailplane, then fin
              -0.18*b  0.18*b  NaN        0       0;
                    0       0  NaN        0 -0.15*b ];
end

function p = ned2plot(v)
% NED is right for the dynamics but awkward to look at, so plot east, north, altitude.
p = [v(2,:); v(1,:); -v(3,:)];
end

function out = clampToLimits(value, limits)
out = min(max(value, limits(1)), limits(2));
end


%% ------------------------------------------------------------- physics
function [x, integ, log] = simStep(x, integ, params, dt)
    % One pass of the mainAirplane.m loop: air data, autopilot, equations of motion.
    air = airData(x, params);
    [delta, integ] = flightController(x, air, integ, params);
    [x_dot, F, M] = dynamics(x, delta, air, params);

    x = x + x_dot*dt; % Euler integration
    x(7:9,1) = wrapToPi(x(7:9,1)); % keep the Euler angles in (-pi,pi]

    log.air = air;
    log.delta = delta;
    log.F = F;
    log.M = M;
end

function air = airData(x, params)
    % Airspeed, incidence angles and dynamic pressure, all from the body velocity.
    v = x(4:6,1);
    h = -x(3,1);

    air.V = max(norm(v), 1e-3); % floored so a standstill cannot divide by zero
    air.alpha = atan2(v(3), v(1));
    air.beta = asin(min(max(v(2)/air.V, -1), 1));
    air.rho = density(h, params);
    air.qbar = 0.5*air.rho*air.V^2;
end

function rho = density(h, params)
    % ISA troposphere, recomputed every step rather than frozen at the starting altitude.
    T = params.env.T0 - params.env.lapse*min(max(h,0), 11000);
    exponent = params.env.g(3)/(params.env.lapse*params.env.R) - 1;
    rho = params.env.rho0*(T/params.env.T0)^exponent;
end

function T = propThrust(throttle, params)
    % Direct force in newtons, interpolated on the curve from flightSim_createAircraft.m.
    T = interp1(params.prop.throttle, params.prop.thrust, min(max(throttle,0),1), 'linear');
end

function [F, M] = aeroForcesMoments(x, delta, air, params)
    % Body-axis coefficient build-up, then dimensionalise.
    w = x(10:12,1);
    a = params.aero;
    S = params.geom.S; b = params.geom.b; c = params.geom.c;

    pb2V = w(1)*b/(2*air.V);
    qcV  = w(2)*c/air.V;
    rb2V = w(3)*b/(2*air.V);

    alpha = air.alpha;
    beta = air.beta;
    da = delta.aileron;
    de = delta.elevator;
    dr = delta.rudder;

    CX = a.CX_0 + a.CX_a*alpha + a.CX_qcV*qcV;
    CY = a.CY_b*beta + a.CY_pb2V*pb2V + a.CY_rb2V*rb2V + a.CY_da*da + a.CY_dr*dr;
    CZ = a.CZ_0 + a.CZ_a*alpha + a.CZ_qcV*qcV + a.CZ_de*de;

    Cl = a.Cl_b*beta + a.Cl_pb2V*pb2V + a.Cl_rb2V*rb2V + a.Cl_da*da + a.Cl_dr*dr;
    Cm = a.Cm_0 + a.Cm_a*alpha + a.Cm_qcV*qcV + a.Cm_rb2V*rb2V + a.Cm_de*de;
    Cn = a.Cn_0 + a.Cn_b*beta + a.Cn_pb2V*pb2V + a.Cn_rb2V*rb2V + a.Cn_da*da + a.Cn_dr*dr;

    F = air.qbar*S*[CX; CY; CZ] + [propThrust(delta.throttle, params); 0; 0];
    M = air.qbar*S*[b*Cl; c*Cm; b*Cn];
end

function [x_dot, F, M] = dynamics(x, delta, air, params)
    v = x(4:6,1); % velocity, body frame
    eul = x(7:9,1); % roll, pitch, yaw
    w = x(10:12,1); % body angular rate

    m = params.body.m;
    g = params.env.g;
    J = params.body.J;

    R = eul2dcm(eul); % body to local
    [F, M] = aeroForcesMoments(x, delta, air, params);

    p_dot = R*v;
    v_dot = -cross(w,v) + F/m + R'*g;
    eul_dot = eulerRates(eul)*w;
    w_dot = J\(M - cross(w,J*w));

    x_dot = [p_dot; v_dot; eul_dot; w_dot];
end

function [delta, integ] = flightController(x, air, integ, params)
    % The example autopilots from flightController.m, in that file's internal sign
    % convention where the surface command is the negative of the loop output.
    z = x(3,1);
    u = x(4,1);
    w = x(6,1);
    phi = x(7,1);
    theta = x(8,1);
    psi = x(9,1);
    p = x(10,1);
    q = x(11,1);
    r = x(12,1);
    beta = air.beta;

    ua_in = -params.pilot.aileron;
    ue_in = -params.pilot.elevator;
    ur_in = -params.pilot.rudder;
    ut_in =  params.pilot.throttle;

    switch params.ctrl.mode
        case 'manual'
            ua = ua_in;
            ue = ue_in;
            ur = ur_in;
            ut = ut_in;

        case 'rateSAS'
            k = params.ctrl.rateSAS;
            ua = -k.Kp*p + ua_in;
            ue = -k.Kq*q + ue_in;
            ur = -k.Kr*r + ur_in;
            ut = ut_in;

        case 'pitchCAS'
            k = params.ctrl.pitchCAS;
            ut = k.throttle;
            e_theta = k.theta_ref - theta;
            integ.theta = clamp(integ.theta + k.Ki*e_theta, params.ctrl.iLimit_theta);
            ue = k.Kp*e_theta - k.Kd*q + integ.theta;
            ua = -p + ua_in;
            ur = -r + ur_in;

        case 'altitude'
            k = params.ctrl.altitude;
            h = -z;
            h_err = clamp(k.Kp_h*(k.h_ref - h) - k.Kd_h*(-w), k.thetaMax);
            e_theta = h_err - theta;
            integ.theta = clamp(integ.theta + k.Ki*e_theta, params.ctrl.iLimit_theta);
            ue = k.Kp*e_theta - k.Kd*q + integ.theta;
            ut = min(max(k.Kt*(k.h_ref - h), 0), 1);
            ua = -p + ua_in;
            ur = -r + ur_in;

        case 'rollYawSAS'
            k = params.ctrl.rollYawSAS;
            ua = k.Kpp*(k.phi_ref - phi) + k.Kpd*(k.p_ref - p) + ua_in;
            ur = k.Kr*(k.r_ref - r) + ur_in;
            ue = -q + ue_in;
            ut = ut_in;

        case 'bankHeading'
            k = params.ctrl.bankHeading;
            ua_ur = -k.K*[0 - beta; 0 - p; 0 - r; k.phi_ref - phi; k.psi_ref - psi];
            ua = ua_ur(1);
            ur = ua_ur(2);
            ue = -q + ue_in;
            ut = ut_in;

        case 'velocity'
            k = params.ctrl.velocity;
            e_u = k.u_ref - u;
            integ.u = clamp(integ.u + k.Ki_u*e_u, k.iLimit_u);
            ut = min(max(k.Kp_u*e_u + integ.u, 0), 1);

            e_theta = k.theta_ref - theta;
            integ.theta = clamp(integ.theta + k.Ki*e_theta, params.ctrl.iLimit_theta);
            ue = k.Kp*e_theta - k.Kd*q + integ.theta;
            ua = -p + ua_in;
            ur = -r + ur_in;

        case 'cruise'
            k = params.ctrl.cruise;
            h = -z;
            e_u = k.u_ref - u;
            integ.u = clamp(integ.u + k.Ki_u*e_u, k.iLimit_u);
            ut = min(max(k.Kp_u*e_u + integ.u, 0), 1);

            h_err = clamp(k.Kp_h*(k.h_ref - h) - k.Kd_h*(-w), k.thetaMax);
            e_theta = h_err - theta;
            integ.theta = clamp(integ.theta + k.Ki*e_theta, params.ctrl.iLimit_theta);
            ue = k.Kp*e_theta - k.Kd*q + integ.theta;
            ua = -p + ua_in;
            ur = -r + ur_in;

        otherwise
            error('mainAirplaneGUI:mode','unknown autopilot mode "%s"', params.ctrl.mode);
    end

    delta.aileron  = clamp(-ua, params.act.aileronMax);
    delta.elevator = clamp(-ue, params.act.elevatorMax);
    delta.rudder   = clamp(-ur, params.act.rudderMax);
    delta.throttle = min(max(ut, 0), 1);
end

function [x0, pilot] = initialState(name, AltitudeMSL, trimSpeed, params)
    % The three conditions from flightSim_createAircraft.m, plus a solved trim.
    switch name
        case 'trimmed'
            [x0, pilot] = trimLevelFlight(trimSpeed, AltitudeMSL, params);

        case 'doNothing'
            x0 = [0; 0; -AltitudeMSL; 70; 0; 0; 0; 0; 0; 0; 0; 0];
            pilot = params.pilot;

        case 'spiral'
            x0 = [0; 0; -AltitudeMSL; 44.54; 2.714; 5.836; 0; 0.1309; 0; 0; 0; 0];
            pilot = struct('aileron',0.1,'elevator',-0.1,'rudder',0,'throttle',0.5);

        case 'trimCoupling'
            x0 = [0; 0; -AltitudeMSL; ...
                  36.911969266574350; 8.107208632644118; 8.825918825964060; ...
                  0.128665353088755; 0.259337659945870; -0.178947917154394; 0; 0; 0];
            pilot = struct('aileron',-0.104711430293605,'elevator',-0.069872522954441, ...
                           'rudder',0,'throttle',0.5);

        otherwise
            error('mainAirplaneGUI:initialCondition','unknown initial condition "%s"', name);
    end
end

function [x0, pilot] = trimLevelFlight(V, h, params)
    % Solve for straight and level flight: three unknowns, angle of attack, elevator and
    % throttle, against no axial acceleration, no normal acceleration, no pitching moment.
    guess = [0.05; -0.05; 0.5];
    step = 1e-6;

    for iteration = 1:60
        residual = trimResidual(guess, V, h, params);
        if norm(residual) < 1e-9
            break
        end
        jacobian = zeros(3,3);
        for k = 1:3
            nudged = guess;
            nudged(k) = nudged(k) + step;
            jacobian(:,k) = (trimResidual(nudged, V, h, params) - residual)/step;
        end
        guess = guess - jacobian\residual;
    end

    alpha = guess(1);
    x0 = [0; 0; -h; V*cos(alpha); 0; V*sin(alpha); 0; alpha; 0; 0; 0; 0];
    pilot = struct('aileron',0,'elevator',guess(2),'rudder',0,'throttle',guess(3));
end

function residual = trimResidual(guess, V, h, params)
    alpha = guess(1);
    x = [0; 0; -h; V*cos(alpha); 0; V*sin(alpha); 0; alpha; 0; 0; 0; 0];
    delta = struct('aileron',0,'elevator',guess(2),'rudder',0,'throttle',guess(3));

    air = airData(x, params);
    x_dot = dynamics(x, delta, air, params);

    residual = [x_dot(4); x_dot(6); x_dot(11)];
end

function out = clamp(value, limit)
    out = min(max(value, -limit), limit);
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


%% ------------------------------------------------------------- setup
function params = defaultParams()
    % Identical to the parameter section of mainAirplane.m.
    params.env.g = [0; 0; 9.80665];
    params.env.rho0 = 1.225;
    params.env.T0 = 288.15;
    params.env.lapse = 0.0065;
    params.env.R = 287.05;

    params.body.m = 2288.231;
    params.body.J = [ 5788.0     0.0  -117.6;
                         0.0  6928.9     0.0;
                      -117.6     0.0 11578.3];

    params.geom.S = 23.2300;
    params.geom.b = 14.6300;
    params.geom.c = 1.5875;

    params.aero.CX_0    = -0.0355;
    params.aero.CX_a    =  0.0029;
    params.aero.CX_qcV  = -0.6748;

    params.aero.CY_b    = -0.7678;
    params.aero.CY_pb2V = -0.1240;
    params.aero.CY_rb2V =  0.3666;

    params.aero.CZ_0    = -0.0550;
    params.aero.CZ_a    = -5.5780;
    params.aero.CZ_qcV  = -2.9880;

    params.aero.Cl_b    = -0.0618;
    params.aero.Cl_pb2V = -0.5045;
    params.aero.Cl_rb2V =  0.1695;

    params.aero.Cm_0    =  0.0945;
    params.aero.Cm_a    = -0.6028;
    params.aero.Cm_qcV  =  0;
    params.aero.Cm_rb2V = -0.3118;

    params.aero.Cn_0    =  0;
    params.aero.Cn_b    =  0.0067;
    params.aero.Cn_pb2V = -0.1585;
    params.aero.Cn_rb2V = -0.1112;

    params.aero.CZ_de = -0.3980;
    params.aero.Cm_de = -1.9210;

    params.aero.Cl_da = -0.0992;
    params.aero.Cn_da = -0.0039;
    params.aero.CY_da = -0.0296;

    params.aero.Cl_dr =  0.0069;
    params.aero.Cn_dr = -0.0827;
    params.aero.CY_dr =  0.1158;

    % The thrust table is generated from its amplitude so the curve can be resized from a
    % single box rather than fourteen. scale is the 5000 in flightSim_createAircraft.m.
    params.prop.throttle = (1/6)*(3 + (-3:3));
    params.prop.scale = 5000;
    params.prop.thrust = params.prop.scale*(1 + tanh(-3:3));

    params.sim.dt = 0.02;

    params.act.aileronMax  = deg2rad(30);
    params.act.elevatorMax = deg2rad(30);
    params.act.rudderMax   = deg2rad(60);

    params.ctrl.mode = 'manual';

    params.pilot.aileron  = 0;
    params.pilot.elevator = 0;
    params.pilot.rudder   = 0;
    params.pilot.throttle = 0.5;

    params.ctrl.rateSAS.Kp = 1;
    params.ctrl.rateSAS.Kq = 5;
    params.ctrl.rateSAS.Kr = 1;

    params.ctrl.pitchCAS.theta_ref = deg2rad(20);
    params.ctrl.pitchCAS.Kp = 1;
    params.ctrl.pitchCAS.Ki = 0.01;
    params.ctrl.pitchCAS.Kd = 1;
    params.ctrl.pitchCAS.throttle = 1;

    params.ctrl.altitude.h_ref = 3000;
    params.ctrl.altitude.Kp_h = 0.01;
    params.ctrl.altitude.Kd_h = 0.01;
    params.ctrl.altitude.thetaMax = deg2rad(15);
    params.ctrl.altitude.Kp = 3;
    params.ctrl.altitude.Ki = 0.01;
    params.ctrl.altitude.Kd = 2;
    params.ctrl.altitude.Kt = 0.1;

    params.ctrl.rollYawSAS.phi_ref = deg2rad(30);
    params.ctrl.rollYawSAS.p_ref = 0;
    params.ctrl.rollYawSAS.r_ref = deg2rad(1);
    params.ctrl.rollYawSAS.Kpp = 1;
    params.ctrl.rollYawSAS.Kpd = 1;
    params.ctrl.rollYawSAS.Kr = 1;

    params.ctrl.bankHeading.phi_ref = deg2rad(30);
    params.ctrl.bankHeading.psi_ref = deg2rad(10);
    params.ctrl.bankHeading.K = [-0.0169  -0.5896  -0.0929  -1.0566  -0.2280;
                                  0.3838   0.0128  -1.1973  -0.2366  -0.9737];

    params.ctrl.velocity.u_ref = 55;
    params.ctrl.velocity.Kp_u = 2;
    params.ctrl.velocity.Ki_u = 1;
    params.ctrl.velocity.iLimit_u = 500;
    params.ctrl.velocity.theta_ref = deg2rad(0);
    params.ctrl.velocity.Kp = 1;
    params.ctrl.velocity.Ki = 0.01;
    params.ctrl.velocity.Kd = 1;

    params.ctrl.cruise.h_ref = 3000;
    params.ctrl.cruise.u_ref = 55;
    params.ctrl.cruise.Kp_u = 2;
    params.ctrl.cruise.Ki_u = 1;
    params.ctrl.cruise.iLimit_u = 500;
    params.ctrl.cruise.Kp_h = 0.01;
    params.ctrl.cruise.Kd_h = 0.01;
    params.ctrl.cruise.thetaMax = deg2rad(15);
    params.ctrl.cruise.Kp = 3;
    params.ctrl.cruise.Ki = 0.01;
    params.ctrl.cruise.Kd = 2;

    params.ctrl.iLimit_theta = 1;
end
