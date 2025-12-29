clear; clc; close all

%% Simulation Setup
Tf = 10; % sec
dt = 0.01;
tspan = 0:dt:Tf;
itr = 1;

%% Initialization
pos      = zeros(3,length(tspan));
vel      = zeros(3,length(tspan));
att      = zeros(3,length(tspan));
rate     = zeros(3,length(tspan));
pos_dot  = zeros(3,length(tspan));
vel_dot  = zeros(3,length(tspan));
att_dot  = zeros(3,length(tspan));
rate_dot = zeros(3,length(tspan));

%% Parameters
m = 10;         % mass, kg
J = 10*eye(3);  % moment of inertia, kg-m^2
g = [0;0;9.81]; % gravity m/s^2
Forces  = [2000; 0; -m*9.81] .* ones(3,length(tspan)); % body-frame, N
Moments = [1; 0; 0] .* ones(3,length(tspan)); % body-frame, N-m

%% Run Simulation
for t = tspan
    % Extract States
    x     = pos(1,itr);  % position in x-axis
    y     = pos(2,itr);  % position in y-axis
    z     = pos(3,itr);  % position in z-axis
    u     = vel(1,itr);  % velocity in x-axis
    v     = vel(2,itr);  % velocity in y-axis
    w     = vel(3,itr);  % velocity in z-axis
    phi   = att(1,itr);  % roll angle, angular position around x-axis
    theta = att(2,itr);  % pitch angle, angular position around y-axis
    psi   = att(3,itr);  % yaw angle, angular position around z-axis
    p     = rate(1,itr); % angular velocity around x-axis
    q     = rate(2,itr); % angular velocity around y-axis
    r     = rate(3,itr); % angular velocity around z-axis

    % Rigid-Body Dynamics       
    C_b2i = [ cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
              cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
             -sin(theta)          sin(phi)*cos(theta)                            cos(phi)*cos(theta)]; % body to inertia
    C_euler = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
               0 cos(phi)           -sin(phi);
               0 sin(phi)/cos(theta) cos(phi)/cos(theta)]; % for euler kinematics
    
    pos_dot(:,itr)  = C_b2i*vel(:,itr);
    vel_dot(:,itr)  = -cross(rate(:,itr),vel(:,itr)) + Forces(:,itr)/m + C_b2i'*g;
    att_dot(:,itr)  = C_euler*rate(:,itr);
    rate_dot(:,itr) = J\(Moments(:,itr)-cross(rate(:,itr),J*rate(:,itr)));

    % Numerical Integration
    if t < tspan(end)
        pos(:,itr+1)  = pos(:,itr) + pos_dot(:,itr)*dt;
        vel(:,itr+1)  = vel(:,itr) + vel_dot(:,itr)*dt;
        att(:,itr+1)  = wrapToPi(att(:,itr) + att_dot(:,itr)*dt);
        rate(:,itr+1) = rate(:,itr) + rate_dot(:,itr)*dt;
        itr = itr + 1;
    end
end

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
plot(tspan, att(1,:), '-r', tspan, att(2,:), '-g', tspan, att(3,:), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('angle (rad)');
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
trajectory_plot(pos, att, 100, 100, 'cessna');

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