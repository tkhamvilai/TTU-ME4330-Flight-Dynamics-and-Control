close all

%% Take the results
tspan = out.tout';
pos(1,:)  = out.SimOut.Xe.Data';
pos(2,:)  = out.SimOut.Ye.Data';
pos(3,:)  = out.SimOut.Ze.Data';
vel(1,:)  = out.SimOut.u.Data';
vel(2,:)  = out.SimOut.v.Data';
vel(3,:)  = out.SimOut.w.Data';
att(1,:)  = out.SimOut.phi.Data';
att(2,:)  = out.SimOut.theta.Data';
att(3,:)  = out.SimOut.psi.Data';
rate(1,:) = out.SimOut.p.Data';
rate(2,:) = out.SimOut.q.Data';
rate(3,:) = out.SimOut.r.Data';

Forces(1,:) = out.SimOut.X.Data';
Forces(2,:) = out.SimOut.Y.Data';
Forces(3,:) = out.SimOut.Z.Data';
Moments(1,:) = out.SimOut.L.Data';
Moments(2,:) = out.SimOut.M.Data';
Moments(3,:) = out.SimOut.N.Data';

%% Plot
subplot(3,2,1);
plot(tspan, pos(1,:), '-r', tspan, pos(2,:), '-g', tspan, pos(3,:), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('position (m)');
legend('x','y','z')

subplot(3,2,2);
plot(tspan, vel(1,:), '-r', tspan, vel(2,:), '-g', tspan, vel(3,:), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('velocity (m)');
legend('u','v','w')

subplot(3,2,3);
plot(tspan, att(1,:)*180/pi, '-r', tspan, att(2,:)*180/pi, '-g', tspan, att(3,:)*180/pi, '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('angle (deg)');
legend('\phi','\theta','\psi')

subplot(3,2,4);
plot(tspan, rate(1,:), '-r', tspan, rate(2,:), '-g', tspan, rate(3,:), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('angular velocity (rad/s)');
legend('p','q','r')

subplot(3,2,5);
plot(tspan, Forces(1,:), '-r', tspan, Forces(2,:), '-g', tspan, Forces(3,:), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('forces (N)');
legend('X','Y','Z')

subplot(3,2,6);
plot(tspan, Moments(1,:), '-r', tspan, Moments(2,:), '-g', tspan, Moments(3,:), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('moment (N-m)');
legend('L','M','N')

figure;
% position, attitude, size_scale_factor, plot_step, model_selector
trajectory_plot(pos, att, 100, 50, 'cessna');
% trajectory_plot(pos, [att(1,:)*0; att(2,:)*0; att(3,:)], 100, 20, 'cessna');

%% Animation
h = Aero.Animation; % create MATLAB animation object
h.createBody('pa24-250_orange.ac','Ac3d'); % https://www.mathworks.com/help/aerotbx/ug/ac3d-files-and-thumbnails-overview.html
h.Bodies{1}.TimeSeriesSource = [tspan' pos' att']; % assign the simulation result to the animation object

% setup camera
% h.Camera.ViewAngle = 5;
% h.Camera.ViewExtent = h.Camera.ViewExtent*10;
% h.Camera.Offset = h.Camera.Offset*10;
h.Camera.PositionFcn = @cameraChaser;

% run the animation
h.initialize();
h.show();
h.play();