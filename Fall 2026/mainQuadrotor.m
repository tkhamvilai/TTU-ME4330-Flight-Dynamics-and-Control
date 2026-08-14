clc; clear; close all
%% Quadcopter simulation: dynamics, cascaded controller, and plots in one file.
%
% AscTec Pelican, Stevens/Lewis/Johnson Table 8.6-1 p. 654. Imperial units throughout
% (ft, slug, lb, s). Local frame is NED, so positive z is down and -30 ft of z is 30 ft up.
%
% States (20). Velocity is in the BODY frame, attitude is Euler angles:
%    1:3    pos       position in the local NED frame, ft
%    4:6    vel       velocity in the body frame, ft/s
%    7:9    eul       roll, pitch, yaw, rad
%   10:12   rate      body angular rate, rad/s
%   13:16   theta     motor phase angle, rad
%   17:20   theta_dot motor angular velocity, rad/s
%
% The control is what a flight controller actually sends: four ESC pulse widths in
% microseconds, 1000 for zero throttle and 2000 for full. The controller converts its
% desired motor speeds into pulse width and saturates there; the plant decodes pulse width
% back into a commanded speed and feeds it through the motor lag.
%
% The motor phase angle is carried but not used.

%% Parameters
% Everything the vehicle and the controller need, grouped by what it describes. 
% Nothing below this section should contain a tuning number.

% ---------------------------------- Environment ----------------------------------
params.env.g = [0; 0; 32.17]; % gravity, ft/s^2
params.env.rho = 0.002377; % sea level air density, slug/ft^3

% ------------------------------------- Body --------------------------------------
params.body.W = 2.8; % weight, lb
params.body.m = params.body.W/params.env.g(3); % mass, slug
params.body.J = 0.032*eye(3); % inertia matrix, slug-ft^2

% --------------------------------- Rotor geometry --------------------------------
params.rotor.n = 4; % number of rotors
params.rotor.l = 1; % arm length, ft
% propeller location relative to C.G., ft
params.rotor.loc = [ params.rotor.l*sin(pi/4)  params.rotor.l*cos(pi/4) 0;   % front-right
                    -params.rotor.l*sin(pi/4)  params.rotor.l*cos(pi/4) 0;   % back-right
                    -params.rotor.l*sin(pi/4) -params.rotor.l*cos(pi/4) 0;   % back-left
                     params.rotor.l*sin(pi/4) -params.rotor.l*cos(pi/4) 0]'; % front-left
params.rotor.dir = [-1 1 -1 1]; % 1:CW, -1:CCW

% ------------------------------- Rotor aerodynamics ------------------------------
params.rotor.kt = 1.2434e-05; % thrust coefficient
params.rotor.torque_to_thrust_ratio = 0.1; % torque to thrust ratio
params.rotor.kq = params.rotor.kt*params.rotor.torque_to_thrust_ratio; % torque coefficient

% ------------------------------- Motor and ESC -----------------------------------
% The controller output is a pulse width and the ESC maps it linearly to a commanded motor
% speed. Full throttle gives 4*kt*omegaMax^2 = 7.96 lb, a thrust-to-weight of 2.8.
params.motor.tau = 0.01; % motor/ESC time constant, s, from Vitor's thesis
params.motor.Jp = 0.00003; % inertia of rotor about spin axis, slug-ft^2
params.motor.torqueMax = 0.21; % maximum motor torque, ft-lb
params.motor.omegaMax = 400; % motor speed at full throttle, rad/s
params.motor.pwmMin = 1000; % pulse width at zero throttle, us
params.motor.pwmMax = 2000; % pulse width at full throttle, us

% ---------------------------------- Mixer ----------------------------------------
% [F;M] = mixer*omega^2, body frame. Row 1 is negative because thrust points along -z.
params.mixer = [-ones(1,params.rotor.n);
                -params.rotor.loc(2,:);
                 params.rotor.loc(1,:);
                -params.rotor.dir].*[params.rotor.kt*ones(3,1); params.rotor.kq];

% -------------------------------- Controller -------------------------------------
% Outer loop: position error -> acceleration command. 
% The ratio Kp/Kd forms the velocity command from position error, 
% then Kd turns the total velocity error into acceleration,
% so Kd alone sets how hard the loop pulls on a velocity error.
params.ctrl.Kp_ol = [1; 1; 1]; % 1/s^2
params.ctrl.Ki_ol = [0.1; 0.1; 0.1]; % 1/s^3
params.ctrl.Kd_ol = [1; 1; 1]; % 1/s
params.ctrl.integral_limit_ol = 1; % anti-windup clamp on the position integral, ft-s
params.ctrl.vel_max = 5; % velocity command saturation, ft/s
params.ctrl.att_limit = deg2rad(45); % how far the thrust axis may lean from vertical, rad

% Inner loop: attitude error -> moment command in ft-lb
params.ctrl.Kp_il = [0.3; 0.3; 0.3]; % ft-lb per rad
params.ctrl.Ki_il = [0.1; 0.1; 0.1]; % ft-lb per rad-s
params.ctrl.Kd_il = [0.15; 0.15; 0.15]; % ft-lb per rad/s
params.ctrl.integral_limit_il = 1.0; % anti-windup clamp on the attitude integral, rad-s

%% Sim Variables
dt = 0.01; % time step
Tf = 30; % time horizon
tspan = 0:dt:Tf;
N_states = 20; % the number of states
x0 = zeros(N_states,1); % inital state

x_hist = zeros(length(tspan),N_states);
x_dot_hist = zeros(length(tspan),N_states);
pwm_hist = zeros(length(tspan),params.rotor.n);
F_hist = zeros(length(tspan),3);
M_hist = zeros(length(tspan),3);
eul_cmd_hist = zeros(length(tspan),3);
att_err_hist = zeros(length(tspan),3);
rate_err_hist = zeros(length(tspan),3);
integral_il_hist = zeros(length(tspan),3);
control_des_hist = zeros(length(tspan),4);

%% Simulation
i = 1; % the number of iterations
x = x0; % inital state
integral_ol = zeros(3,1); % outerloop integrator state, ft-s
integral_il = zeros(3,1); % innerloop integrator state, rad-s
ctrl = params.ctrl; % shorthand, keeps the control equations below readable
for t = tspan
    % Desired states
    pos_des = [10;20;-30]; % local frame, negative z up
    vel_des = [0;0;0]; % local frame
    eul_des = wrapToPi(deg2rad([0;0;45])); % desired phi theta psi
    rate_des = [0;0;0]; % desired angular rate (body frame)

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
    pos_err = rot_horizon*(pos_des - pos); % position error in body frame (horizontally)
    vel_err = rot_horizon*(vel_des - vel_local); % velocity error in body frame (horizontally)

    % Position Controller
    vp_ol = (ctrl.Kp_ol./ctrl.Kd_ol).*pos_err;
    if norm(vp_ol) > ctrl.vel_max
        vp_ol = vp_ol*ctrl.vel_max/norm(vp_ol);
    end
    vpd_ol = ctrl.Kd_ol.*(vp_ol + vel_err);
    integral_ol = min(max(integral_ol + dt*pos_err, -ctrl.integral_limit_ol),ctrl.integral_limit_ol);
    vi_ol = ctrl.Ki_ol.*integral_ol;
    vpid_ol = vpd_ol + vi_ol; % outerloop acceleration command

    % Acceleration Command to Desired Attitude
    spf = vpid_ol - gB; % specific force in body frame
    F_des = spf(3)*params.body.m; % only the third component matters since all motors point upward

    sdes = vpid_ol; % sdes will be a unit acceleration vector that includes gravity because we want to rotate this vector
    sdes(3) = -params.env.g(3) + min(0,sdes(3)); % for z-axis add gravity (in the opposite direction) to sdes, this will be a rotation around hover
    sdes = -sdes/max(norm(sdes),0.01); % flip sdes to point in the gravity direction, this is the desired body z axis
    yaw_error = wrapToPi(eul_des(3) - eul(3));
    rot_horizon_yaw_error = [ cos(yaw_error) sin(yaw_error) 0;
                             -sin(yaw_error) cos(yaw_error) 0;
                              0 0 1];
    ades = rot_horizon_yaw_error*sdes; % to make roll & pitch follow yaw

    % Limit how far the desired body z axis may lean away from straight down
    ol_mu = acos(min(max(ades(3),-1),1)); % angle between ades and [0;0;1]
    if ol_mu > ctrl.att_limit
        ades = [sin(ctrl.att_limit)*ades(1:2)/max(norm(ades(1:2)),0.01); cos(ctrl.att_limit)];
    end

    % Desired Euler angles from the desired body z axis. Expressed in the yaw-aligned frame
    % that axis is [cos(phi)*sin(theta); -sin(phi); cos(phi)*cos(theta)], so roll and pitch
    % fall straight out of its components.
    phi_des = -asin(min(max(ades(2),-1),1));
    theta_des = atan2(ades(1),ades(3));
    eul_cmd = [phi_des; theta_des; eul_des(3)];

    % Innerloop errors
    att_err = eulerRates(eul)\wrapToPi(eul_cmd - eul); % Euler error mapped to body axes
    rate_err = rate_des - rate; % angular rate error

    % Attitude Controller
    % The PID produces the moment command directly, in ft-lb.
    integral_il = min(max(integral_il + dt*att_err, -ctrl.integral_limit_il),ctrl.integral_limit_il);
    M_des = ctrl.Kp_il.*att_err + ctrl.Ki_il.*integral_il + ctrl.Kd_il.*rate_err; % PID

    control_des = [F_des; M_des];

    % Mixer inverse gives the motor speed each rotor would have to hold
    U_sq = max(pinv(params.mixer)*control_des,0);
    omega_des = sqrt(U_sq);

    % Convert to the signal actually sent to the ESCs. Saturating here, in pulse width, is
    % what makes motor saturation physical: a command past full throttle is simply clipped.
    pwm = params.motor.pwmMin + (params.motor.pwmMax - params.motor.pwmMin)*omega_des/params.motor.omegaMax;
    pwm = min(max(pwm,params.motor.pwmMin),params.motor.pwmMax);

    % Quadrotor Dynamics
    [x_dot,F,M] = dynamics(x,pwm,params); % EoM function call
    x = x + x_dot*dt; % Euler integration
    x(7:9,1) = wrapToPi(x(7:9,1)); % keep the Euler angles in (-pi,pi]

    % save results for plotting
    x_hist(i,:) = x';
    x_dot_hist(i,:) = x_dot';
    F_hist(i,:) = F';
    M_hist(i,:) = M';
    pwm_hist(i,:) = pwm';
    eul_cmd_hist(i,:) = eul_cmd';
    att_err_hist(i,:) = rad2deg(att_err)';
    rate_err_hist(i,:) = rate_err';
    integral_il_hist(i,:) = integral_il';
    control_des_hist(i,:) = control_des';
    i = i + 1;
end

%% Plot
figure('Name','Quadcopter states','Color','w');

subplot(3,2,1);
plot(tspan, x_hist(:,1), '-r', tspan, x_hist(:,2), '-g', tspan, x_hist(:,3), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('position (ft)');
legend('x','y','z','Location','best')

subplot(3,2,2);
plot(tspan, x_hist(:,4), '-r', tspan, x_hist(:,5), '-g', tspan, x_hist(:,6), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('body velocity (ft/s)');
legend('u','v','w','Location','best')

subplot(3,2,3);
plot(tspan, rad2deg(x_hist(:,7)), '-r', tspan, rad2deg(x_hist(:,8)), '-g', tspan, rad2deg(x_hist(:,9)), '-b');
hold on
plot(tspan, rad2deg(eul_cmd_hist(:,1)), '--r', tspan, rad2deg(eul_cmd_hist(:,2)), '--g', tspan, rad2deg(eul_cmd_hist(:,3)), '--b');
hold off
grid on; grid minor
xlabel('time (s)'); ylabel('attitude (deg)');
legend('\phi','\theta','\psi','Location','best')
title('dashed = commanded')

subplot(3,2,4);
plot(tspan, rad2deg(x_hist(:,10)), '-r', tspan, rad2deg(x_hist(:,11)), '-g', tspan, rad2deg(x_hist(:,12)), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('body rate (deg/s)');
legend('p','q','r','Location','best')

subplot(3,2,5);
plot(tspan, pwm_hist);
hold on
yline(params.motor.pwmMin, 'k--'); yline(params.motor.pwmMax, 'k--');
hold off
grid on; grid minor
xlabel('time (s)'); ylabel('PWM command (\mus)');
legend('1','2','3','4','Location','best')
title('dashed = throttle rails')

subplot(3,2,6);
plot(tspan, x_hist(:,17:20));
grid on; grid minor
xlabel('time (s)'); ylabel('motor speed (rad/s)');
legend('1','2','3','4','Location','best')

figure('Name','Trajectory','Color','w');
plot3(x_hist(:,2), x_hist(:,1), -x_hist(:,3), '-b', 'LineWidth', 1.5);
hold on
plot3(pos_des(2), pos_des(1), -pos_des(3), 'ro', 'MarkerFaceColor', 'r');
hold off
grid on; axis equal
xlabel('east (ft)'); ylabel('north (ft)'); zlabel('altitude (ft)');
legend('flight path','target','Location','best')
view(45,25);

%% Local Functions
function [x_dot, F, M] = dynamics(x,u,params)
    % States
    % Position x(1:3) and motor phase angle x(13:16) are left out on purpose: neither one
    % affects the derivatives. The phase angle is integrated only so that it is available
    % to a future blade-element rotor model.
    v = x(4:6,1); % velocity, body frame
    eul = x(7:9,1); % roll, pitch, yaw
    w = x(10:12,1); % body-angular rate
    theta_dot = x(17:20,1); % motor angular velocity

    % Controls
    % The ESC decodes pulse width into a commanded motor speed. Clamping here as well as in
    % the controller keeps the plant honest: whatever the controller sends, the motors can
    % only ever be asked for something between zero and full throttle.
    pwm = min(max(u(1:params.rotor.n,1),params.motor.pwmMin),params.motor.pwmMax);
    omega_cmd = params.motor.omegaMax*(pwm - params.motor.pwmMin)/(params.motor.pwmMax - params.motor.pwmMin);

    % Aircraft Variables
    m = params.body.m;
    g = params.env.g;
    J = params.body.J;
    tau = params.motor.tau;

    % Helper Functions
    R = eul2dcm(eul); % body to local

    % rotor
    FM = params.mixer*(theta_dot.^2); % body
    F = [0;0;FM(1,1)]; % body, thrust is along -z so this is negative
    M = FM(2:end,1);

    % Equations of Motion
    p_dot = R*v;
    v_dot = -cross(w,v) + F/m + R'*g;
    eul_dot = eulerRates(eul)*w;
    w_dot = J\(M-cross(w,J*w));
    theta_dot_dot = 1/tau*(omega_cmd - theta_dot); % 8.6-2

    % State Derivatives
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
