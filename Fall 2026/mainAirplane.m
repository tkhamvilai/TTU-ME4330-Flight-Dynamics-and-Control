clc; clear; close all

%% Parameters
% ---------------------------------- Environment ----------------------------------
params.env.g   = 9.81; % gravity, m/s^2
params.env.rho = 1.225; % sea level density, kg/m^3

% ------------------------------------- Body --------------------------------------
params.body.m = 2288.231; % mass, kg
params.body.J = [ 5788.0     0.0  -117.6;      % inertia, kg*m^2
                     0.0  6928.9     0.0;      
                  -117.6     0.0 11578.3];

% --------------------------------- Reference geometry ----------------------------
params.geom.S = 23.2300; % reference area,  m^2
params.geom.b = 14.6300; % reference span,  m
params.geom.c = 1.5875;  % reference chord, m

% ------------------------------ Aerodynamic coefficients -------------------------
params.aero.CX_0    = -0.0355;
params.aero.CX_a    =  0.0029;
params.aero.CX_q    = -0.6748;

params.aero.CY_0    =  0;
params.aero.CY_b    = -0.7678;
params.aero.CY_p    = -0.1240;
params.aero.CY_r    =  0.3666;

params.aero.CZ_0    = -0.0550;
params.aero.CZ_a    = -5.5780;
params.aero.CZ_q    = -2.9880;

params.aero.Cl_0    =  0;
params.aero.Cl_b    = -0.0618;
params.aero.Cl_p    = -0.5045;
params.aero.Cl_r    =  0.1695;

params.aero.Cm_0    =  0.0945;
params.aero.Cm_a    = -0.6028;
params.aero.Cm_q    = -5.4321;

params.aero.Cn_0    =  0;
params.aero.Cn_b    =  0.0067;
params.aero.Cn_p    = -0.1585;
params.aero.Cn_r    = -0.1112;

% Control surface effectiveness
params.aero.CX_de =  0;
params.aero.CZ_de = -0.3980;
params.aero.Cm_de = -1.9210;

params.aero.Cl_da = -0.0992;
params.aero.Cn_da = -0.0039;
params.aero.CY_da = -0.0296;

params.aero.Cl_dr =  0.0069;
params.aero.Cn_dr = -0.0827;
params.aero.CY_dr =  0.1158;

% -------------------------------- Propulsion -------------------------------------
params.prop.throttle = (1/6)*(3 + (-3:0.1:3)); % breakpoints, 0 to 1
params.prop.thrust = 5000*(1 + tanh(-3:0.1:3)); % N, about 25 N at idle and 9975 N at full
params.aero.CX_dt = mean(diff(params.prop.thrust)./diff(params.prop.throttle));
params.aero.CZ_dt = 0;
params.aero.Cm_dt = 0;

% ------------------------------ Actuator limits ----------------------------------
params.act.aileronMax  = deg2rad(30);
params.act.elevatorMax = deg2rad(30);
params.act.rudderMax   = deg2rad(60);

% -------------------------------- Autopilot --------------------------------------
%   'manual'      pilot trim straight through, no feedback
%   'rateSAS'     rate stability augmentation about the trim condition
%   'pitchCAS'    climb holding a constant pitch angle, PID on theta
%   'altitude'    climb to an altitude and level out
%   'rollYawSAS'  turn coordinator / emergency descent
%   'bankHeading' bank and heading hold by LQR full-state feedback, forward-slip landing
%   'velocity'    velocity hold, speed control and gliding
%   'cruise'      altitude hold and velocity hold together
params.ctrl.mode = 'manual';

% manual control
params.pilot.aileron  = 0;
params.pilot.elevator = 0;
params.pilot.rudder   = 0;
params.pilot.throttle = 0.5;

% stability augmentation system
params.ctrl.rateSAS.Kp = 1; % roll rate
params.ctrl.rateSAS.Kq = 5; % pitch rate
params.ctrl.rateSAS.Kr = 1; % yaw rate

% climbing
params.ctrl.pitchCAS.theta_ref = deg2rad(20);
params.ctrl.pitchCAS.Kp = 1;
params.ctrl.pitchCAS.Ki = 0.01;
params.ctrl.pitchCAS.Kd = 1;
params.ctrl.pitchCAS.throttle = 1; % full throttle for the climb

% climbing and leveling out
params.ctrl.altitude.h_ref = 3000; % m
params.ctrl.altitude.Kp_h = 0.01; % altitude error to pitch command
params.ctrl.altitude.Kd_h = 0.01; % climb rate damping
params.ctrl.altitude.thetaMax = deg2rad(15);
params.ctrl.altitude.Kp = 3;
params.ctrl.altitude.Ki = 0.01;
params.ctrl.altitude.Kd = 2;
params.ctrl.altitude.Kt = 0.1; % altitude error to throttle

% emergency descending
params.ctrl.rollYawSAS.phi_ref = deg2rad(30);
params.ctrl.rollYawSAS.p_ref = 0;
params.ctrl.rollYawSAS.r_ref = deg2rad(1);
params.ctrl.rollYawSAS.Kpp = 1;
params.ctrl.rollYawSAS.Kpd = 1;
params.ctrl.rollYawSAS.Kr = 1;

% forward-slip landing
params.ctrl.bankHeading.phi_ref = deg2rad(30);
params.ctrl.bankHeading.psi_ref = deg2rad(10);
params.ctrl.bankHeading.K = [-0.0169  -0.5896  -0.0929  -1.0566  -0.2280;   % aileron row
                              0.3838   0.0128  -1.1973  -0.2366  -0.9737];  % rudder row

% gliding
params.ctrl.velocity.u_ref = 55; % m/s
params.ctrl.velocity.Kp_u = 2;
params.ctrl.velocity.Ki_u = 1;
params.ctrl.velocity.iLimit_u = 500;
params.ctrl.velocity.theta_ref = deg2rad(0);
params.ctrl.velocity.Kp = 1;
params.ctrl.velocity.Ki = 0.01;
params.ctrl.velocity.Kd = 1;

% cruising
params.ctrl.cruise.h_ref = 3000; % m
params.ctrl.cruise.u_ref = 55; % m/s
params.ctrl.cruise.Kp_u = 2;
params.ctrl.cruise.Ki_u = 1;
params.ctrl.cruise.iLimit_u = 500;
params.ctrl.cruise.Kp_h = 0.01;
params.ctrl.cruise.Kd_h = 0.01;
params.ctrl.cruise.thetaMax = deg2rad(15);
params.ctrl.cruise.Kp = 3;
params.ctrl.cruise.Ki = 0.01;
params.ctrl.cruise.Kd = 2;

params.ctrl.iLimit_theta = 1; % clamp on the pitch integrator, shared by every mode

%% Sim Variables
dt = 0.02; % time step, s
Tf = 200; % time horizon, s
tspan = 0:dt:Tf;
N_states = 12; % the number of states

AltitudeMSL = 2202; % operating height, m

% Initial condition (pick one). 'trimmed' solves for straight and level flight at the
% speed below; the other three are the conditions defined in flightSim_createAircraft.m.
% initialCondition = 'doNothing';
initialCondition = 'trimmed';
trimSpeed = 55; % m/s, used by 'trimmed' only

[x0, params.pilot] = initialState(initialCondition, AltitudeMSL, trimSpeed, params);

if strcmp(initialCondition, 'trimmed')
    [A_numerical, B_numerical] = linearizeDynamics(x0, params.pilot, params);
    
    % convert velocity from body to wind
    A_numerical(5,:) = A_numerical(5,:) / trimSpeed;
    B_numerical(5,:) = B_numerical(5,:) / trimSpeed;
    A_numerical(6,:) = A_numerical(6,:) / trimSpeed;
    B_numerical(6,:) = B_numerical(6,:) / trimSpeed;

    longitudinal_states  = [4 6 11 8]; % u w theta q
    longitudinal_control = [2 4]; % elevator and throttle
    A_longitudinal_numerical = A_numerical(longitudinal_states, longitudinal_states);
    B_longitudinal_numerical = B_numerical(longitudinal_states, longitudinal_control);

    lateral_states  = [5 10 12 7]; % v p r phi
    lateral_control = [1 3]; % elevator and throttle
    A_lateral_numerical = A_numerical(lateral_states, lateral_states);
    B_lateral_numerical = B_numerical(lateral_states, lateral_control);

% ------------------------------ Stability Derivatives ---------------------------
QS = 0.5*params.env.rho*trimSpeed^2*params.geom.S;
Jx = params.body.J(1,1);
Jy = params.body.J(2,2);
Jz = params.body.J(3,3);
Jxz = params.body.J(1,3);

% Longitudinal
X_u   = QS/params.body.m/trimSpeed*params.aero.CX_0*(2);
X_a   = QS/params.body.m*(params.aero.CX_a - params.aero.CZ_0*0);
X_q   = QS/params.body.m*params.aero.CX_q*params.geom.c/2/trimSpeed;
X_dt  = QS/params.body.m*params.aero.CX_dt;
X_de  = QS/params.body.m*params.aero.CX_de;

Z_u   = QS/params.body.m/trimSpeed*params.aero.CZ_0*(2);
Z_a   = QS/params.body.m*(params.aero.CZ_a + params.aero.CX_0*0);
Z_q   = QS/params.body.m*params.aero.CZ_q*params.geom.c/2/trimSpeed;
Z_dt  = QS/params.body.m*params.aero.CZ_dt;
Z_de  = QS/params.body.m*params.aero.CZ_de;

M_u   = QS*params.geom.c/Jy/trimSpeed*params.aero.Cm_0*(2);
M_a   = QS*params.geom.c/Jy*params.aero.Cm_a;
M_q   = QS*params.geom.c/Jy/trimSpeed*params.aero.Cm_q*params.geom.c;
M_dt  = QS/params.geom.c/Jy*params.aero.Cm_dt;
M_de  = QS*params.geom.c/Jy*params.aero.Cm_de;

% x = [u; alpha; q; theta]
A_longitudinal_analytical = [X_u           X_a            0 -params.env.g;
                             Z_u/trimSpeed Z_a/trimSpeed  1  0;
                             M_u           M_a           M_q 0;
                              0             0             1  0];
B_longitudinal_analytical = [X_de            X_dt;
                             Z_de/trimSpeed  Z_dt/trimSpeed;
                             M_de            M_dt;
                             0                0];
disp('Longitudinal Stability'); damp(A_longitudinal_analytical)
tspan_stability = 0:0.1:250;
perturbation = [0.01*trimSpeed;0;deg2rad(1);deg2rad(0)];
figure;
lsim(ss(A_longitudinal_analytical,[],eye(4),0),[],tspan_stability,perturbation);
title('Longitudinal Dynamics')

% Phugoid / Long-Period Mode
% x = [u; theta]
A_pg = [X_u -params.env.g; -Z_u/trimSpeed 0];
disp('Phugoid / Long-Period Mode Stability'); damp(A_pg)
tspan_stability = 0:0.1:250;
perturbation = [0.01*trimSpeed;deg2rad(0)];
figure;
lsim(ss(A_pg,[],eye(2),0),[],tspan_stability,perturbation);
title('Phugoid / Long-Period Mode Dynamics')

% Short-Period Mode
% x = [alpha; q]
A_sp = [Z_a/trimSpeed 1; M_a M_q];
disp('Short-Period Mode Stability'); damp(A_sp)
tspan_stability = 0:0.1:25;
perturbation = [0;deg2rad(1)];
figure;
lsim(ss(A_sp,[],eye(2),0),[],tspan_stability,perturbation);
title('Short-Period Mode Dynamics')

% Lateral Dynamics
Y_b  = QS/params.body.m*params.aero.CY_b;
Y_p  = QS/params.body.m/trimSpeed*params.aero.CY_p*(params.geom.b/2);
Y_r  = QS/params.body.m/trimSpeed*params.aero.CY_r*(params.geom.b/2);
Y_da = QS/params.body.m*params.aero.Cn_da;
Y_dr = QS/params.body.m*params.aero.Cn_dr;

L_b  = QS*params.geom.b/Jx*params.aero.Cl_b;
L_p  = QS*params.geom.b/Jx/trimSpeed*params.aero.Cl_p*(params.geom.b/2);
L_r  = QS*params.geom.b/Jx/trimSpeed*params.aero.Cl_r*(params.geom.b/2);
L_da = QS*params.geom.b/Jx*params.aero.Cl_da;
L_dr = QS*params.geom.b/Jx*params.aero.Cl_dr;

N_b  = QS*params.geom.b/Jz*params.aero.Cn_b;
N_p  = QS*params.geom.b/Jz/trimSpeed*params.aero.Cn_p*(params.geom.b/2);
N_r  = QS*params.geom.b/Jz/trimSpeed*params.aero.Cn_r*(params.geom.b/2);
N_da = QS*params.geom.b/Jx*params.aero.Cn_da;
N_dr = QS*params.geom.b/Jx*params.aero.Cn_dr;

% roll-yaw coupling
Coupling      = 1 - Jxz^2/Jx/Jz;
L_b_coupling  = L_b/Coupling;
L_p_coupling  = L_p/Coupling;
L_r_coupling  = L_r/Coupling;
L_da_coupling = L_da/Coupling;
L_dr_coupling = L_dr/Coupling;

N_b_coupling  = N_b/Coupling;
N_p_coupling  = N_p/Coupling;
N_r_coupling  = N_r/Coupling;
N_da_coupling = N_da/Coupling;
N_dr_coupling = N_dr/Coupling;

L_b_prime  = L_b_coupling  + Jxz/Jx*N_b_coupling;
L_p_prime  = L_p_coupling  + Jxz/Jx*N_p_coupling;
L_r_prime  = L_r_coupling  + Jxz/Jx*N_r_coupling;
L_da_prime = L_da_coupling + Jxz/Jx*N_da_coupling;
L_dr_prime = L_dr_coupling + Jxz/Jx*N_dr_coupling;

N_b_prime  = N_b_coupling  + Jxz/Jz*L_b_coupling;
N_p_prime  = N_p_coupling  + Jxz/Jz*L_p_coupling;
N_r_prime  = N_r_coupling  + Jxz/Jz*L_r_coupling;
N_da_prime = N_da_coupling + Jxz/Jz*L_da_coupling;
N_dr_prime = N_dr_coupling + Jxz/Jz*L_dr_coupling;

% x = [beta; p; r; phi]
A_lateral_analytical = [Y_b/trimSpeed     Y_p/trimSpeed    -(1-Y_r/trimSpeed)  params.env.g/trimSpeed;
                        L_b_prime         L_p_prime          L_r_prime         0;
                        N_b_prime         N_p_prime          N_r_prime         0;
                        0 1 0 0];
B_lateral_analytical = [Y_da/trimSpeed     Y_dr/trimSpeed;
                        L_da_prime         L_dr_prime;
                        N_da_prime         N_dr_prime;
                            0                  0];
disp('Lateral Stability'); damp(A_lateral_analytical)
tspan_stability = 0:0.1:250;
perturbation = [deg2rad(1);deg2rad(1);deg2rad(1);deg2rad(0)];
figure;
lsim(ss(A_lateral_analytical,[],eye(4),0),[],tspan_stability,perturbation);
title('Lateral Dynamics')

% Spiral
% x = [r]
A_spi = N_r_prime - L_r_prime*N_b_prime/L_b_prime;
disp('Spiral Stability'); damp(A_spi)
tspan_stability = 0:0.1:25;
perturbation = deg2rad(1);
figure;
lsim(ss(A_spi,[],eye(1),0),[],tspan_stability,perturbation);
title('Spiral Mode Dynamics')

% Roll
% x = [p]
A_roll = L_p_prime;
disp('Roll Mode Stability'); damp(A_roll)
tspan_stability = 0:0.1:25;
perturbation = deg2rad(1);
figure;
lsim(ss(A_roll,[],eye(1),0),[],tspan_stability,perturbation);
title('Roll Mode Dynamics')

%  Dutch-Roll
% x = [beta r]
A_dr = [Y_b/trimSpeed -(1-Y_r/trimSpeed);
        N_b_prime N_r_prime];
disp('Dutch-Roll Mode Stability'); damp(A_dr)
tspan_stability = 0:0.1:25;
perturbation = [deg2rad(1);deg2rad(1)];
figure;
lsim(ss(A_dr,[],eye(2),0),[],tspan_stability,perturbation);
title('Dutch-Roll Mode Dynamics')

end

%% Simulation
x_hist = zeros(length(tspan),N_states);
x_dot_hist = zeros(length(tspan),N_states);
delta_hist = zeros(length(tspan),4); % aileron, elevator, rudder, throttle
air_hist = zeros(length(tspan),3); % V, alpha, beta
force_hist = zeros(length(tspan),3);
moment_hist = zeros(length(tspan),3);

i = 1; % the number of iterations
x = x0; % initial state
integ.theta = 0; % pitch integrator
integ.u = 0; % airspeed integrator
for t = tspan
    % Autopilot
    [control, integ] = flightController(x, integ, params);

    % Equations of motion
    [x_dot, F, M] = dynamics(x, control, params);

    % save results for plotting
    x_hist(i,:) = x';
    x_dot_hist(i,:) = x_dot';
    delta_hist(i,:) = [control.aileron, control.elevator, control.rudder, control.throttle];
    
    force_hist(i,:) = F';
    moment_hist(i,:) = M';

    % Air data here for plotting
    air = airData(x, params);
    air_hist(i,:) = [air.V, air.alpha, air.beta];

    x = x + x_dot*dt; % Euler integration
    x(7:9,1) = wrapToPi(x(7:9,1)); % keep the Euler angles in (-pi,pi]
    i = i + 1;
end

%% Plot
figure('Name',['Airplane states - ' params.ctrl.mode],'Color','w');

subplot(3,3,1);
plot(tspan, x_hist(:,1), '-r', tspan, x_hist(:,2), '-g');
grid on; grid minor
xlabel('time (s)'); ylabel('position (m)');
legend('north','east','Location','best')

subplot(3,3,2);
plot(tspan, -x_hist(:,3), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('altitude (m)');

subplot(3,3,3);
plot(tspan, air_hist(:,1), '-k');
grid on; grid minor
xlabel('time (s)'); ylabel('airspeed (m/s)');

subplot(3,3,4);
plot(tspan, x_hist(:,4), '-r', tspan, x_hist(:,5), '-g', tspan, x_hist(:,6), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('body velocity (m/s)');
legend('u','v','w','Location','best')

subplot(3,3,5);
plot(tspan, rad2deg(x_hist(:,7)), '-r', tspan, rad2deg(x_hist(:,8)), '-g', tspan, rad2deg(x_hist(:,9)), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('attitude (deg)');
legend('\phi','\theta','\psi','Location','best')

subplot(3,3,6);
plot(tspan, rad2deg(x_hist(:,10)), '-r', tspan, rad2deg(x_hist(:,11)), '-g', tspan, rad2deg(x_hist(:,12)), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('body rate (deg/s)');
legend('p','q','r','Location','best')

subplot(3,3,7);
plot(tspan, rad2deg(air_hist(:,2)), '-r', tspan, rad2deg(air_hist(:,3)), '-g');
grid on; grid minor
xlabel('time (s)'); ylabel('angle (deg)');
legend('\alpha','\beta','Location','best')

subplot(3,3,8);
plot(tspan, rad2deg(delta_hist(:,1)), '-r', tspan, rad2deg(delta_hist(:,2)), '-g', tspan, rad2deg(delta_hist(:,3)), '-b');
grid on; grid minor
xlabel('time (s)'); ylabel('deflection (deg)');
legend('aileron','elevator','rudder','Location','best')

subplot(3,3,9);
plot(tspan, delta_hist(:,4), '-k');
grid on; grid minor
xlabel('time (s)'); ylabel('throttle (0-1)');
ylim([-0.05 1.05]);

figure('Name','Trajectory','Color','w');
plot3(x_hist(:,2), x_hist(:,1), -x_hist(:,3), '-b', 'LineWidth', 1.5);
hold on
plot3(x_hist(1,2), x_hist(1,1), -x_hist(1,3), 'go', 'MarkerFaceColor', 'g');
plot3(x_hist(end,2), x_hist(end,1), -x_hist(end,3), 'ro', 'MarkerFaceColor', 'r');
hold off
grid on; axis equal
xlabel('east (m)'); ylabel('north (m)'); zlabel('altitude (m)');
legend('flight path','start','end','Location','best')
view(45,25);

%% Local Functions
function air = airData(x, params)
    % Airspeed, incidence angles and dynamic pressure, all from the body velocity.
    v = x(4:6,1);

    air.V = max(norm(v), 1e-3); % floored so a standstill cannot divide by zero
    air.alpha = atan2(v(3), v(1));
    air.beta = asin(min(max(v(2)/air.V, -1), 1));
    air.qbar = 0.5*params.env.rho*air.V^2;
end

function [F, M] = aeroForcesMoments(x, control, params)
    air = airData(x, params);
    
    % Body-axis coefficient build-up, then dimensionalise.
    p = x(10,1);
    q = x(11,1);
    r = x(12,1);
    a = params.aero;
    S = params.geom.S; 
    b = params.geom.b; 
    c = params.geom.c;
    V = air.V;

    % Non-dimensional rates, using the same normalization
    pb2V = p*b/(2*V);
    qc2V = q*c/(2*V);
    rb2V = r*b/(2*V);

    alpha = air.alpha;
    beta = air.beta;
    da = control.aileron;
    de = control.elevator;
    dr = control.rudder;
    dt = control.throttle;

    CX = a.CX_0 + a.CX_a*alpha + a.CX_q*qc2V + a.CX_de*de;
    CY = a.CY_0 + a.CY_b*beta  + a.CY_p*pb2V + a.CY_r*rb2V + a.CY_da*da + a.CY_dr*dr;
    CZ = a.CZ_0 + a.CZ_a*alpha + a.CZ_q*qc2V + a.CZ_de*de;

    Cl = a.Cl_0 + a.Cl_b*beta  + a.Cl_p*pb2V + a.Cl_r*rb2V + a.Cl_da*da + a.Cl_dr*dr;
    Cm = a.Cm_0 + a.Cm_a*alpha + a.Cm_q*qc2V + a.Cm_de*de;
    Cn = a.Cn_0 + a.Cn_b*beta  + a.Cn_p*pb2V + a.Cn_r*rb2V + a.Cn_da*da + a.Cn_dr*dr;

    Thrust = interp1(params.prop.throttle, params.prop.thrust, clip(dt, 0, 1), 'linear');

    F = air.qbar*S*[CX; CY; CZ] + [Thrust; 0; 0];
    M = air.qbar*S*[b*Cl; c*Cm; b*Cn];
end

function [x_dot, F, M] = dynamics(x, control, params)
    pos = x(1:3); % inertia frame
    vel = x(4:6); % body frame
    att = x(7:9); % inertia frame
    rate = x(10:12); % body frame

    phi   = att(1); % roll angle
    theta = att(2); % pitch angle
    psi   = att(3); % yaw angle

    R_phi = [1  0         0;
             0  cos(phi)  sin(phi);
             0 -sin(phi)  cos(phi)];

    R_theta = [cos(theta)  0   -sin(theta);
               0           1    0;
               sin(theta)  0    cos(theta)];

    R_psi = [ cos(psi)  sin(psi)  0;
             -sin(psi)  cos(psi)  0;
              0             0     1];

    R_bI = R_phi * R_theta * R_psi; % body relative to inertial

    H = [1 sin(phi)*tan(theta)  cos(phi)*tan(theta);
         0 cos(phi)            -sin(phi);
         0 sin(phi)/cos(theta)  cos(phi)/cos(theta)];

    g_vec = [0; 0; params.env.g];

    m = params.body.m;
    g = params.env.g;
    J = params.body.J;

    [F, M] = aeroForcesMoments(x, control, params);

    pos_dot = R_bI' * vel;
    att_dot = H * rate;
    vel_dot = -cross(rate, vel) + R_bI * g_vec + F/m;
    rate_dot = inv(J)*(M - cross(rate, J*rate));
    x_dot = [pos_dot; vel_dot; att_dot; rate_dot];
end

function [delta, integ] = flightController(x, integ, params)
    air = airData(x, params);

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

    % Correct for actuator direction: pilot trim, entering the internal convention
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
            % Rate SAS from the trim condition
            k = params.ctrl.rateSAS;
            ua = -k.Kp*p + ua_in;
            ue = -k.Kq*q + ue_in;
            ur = -k.Kr*r + ur_in;
            ut = ut_in;

        case 'pitchCAS'
            % Pitch CAS (PID), climb at constant pitch angle
            k = params.ctrl.pitchCAS;
            ut = k.throttle;
            e_theta = k.theta_ref - theta;
            integ.theta = clip(integ.theta + k.Ki*e_theta, -params.ctrl.iLimit_theta, params.ctrl.iLimit_theta);
            ue = k.Kp*e_theta - k.Kd*q + integ.theta;
            ua = -p + ua_in;
            ur = -r + ur_in;

        case 'altitude'
            % Altitude, climb and level out
            k = params.ctrl.altitude;
            h = -z;
            h_err = clip(k.Kp_h*(k.h_ref - h) - k.Kd_h*(-w), -k.thetaMax, k.thetaMax);
            e_theta = h_err - theta;
            integ.theta = clip(integ.theta + k.Ki*e_theta, -params.ctrl.iLimit_theta, params.ctrl.iLimit_theta);
            ue = k.Kp*e_theta - k.Kd*q + integ.theta;
            ut = min(max(k.Kt*(k.h_ref - h), 0), 1);
            ua = -p + ua_in;
            ur = -r + ur_in;

        case 'rollYawSAS'
            % Roll-Yaw SAS, turn coordinator / emergency descent
            k = params.ctrl.rollYawSAS;
            ua = k.Kpp*(k.phi_ref - phi) + k.Kpd*(k.p_ref - p) + ua_in;
            ur = k.Kr*(k.r_ref - r) + ur_in;
            ue = -q + ue_in;
            ut = ut_in;

        case 'bankHeading'
            % Bank-Heading CAS by LQR full-state feedback, forward-slip landing
            k = params.ctrl.bankHeading;
            ua_ur = -k.K*[0 - beta; 0 - p; 0 - r; k.phi_ref - phi; k.psi_ref - psi];
            ua = ua_ur(1);
            ur = ua_ur(2);
            ue = -q + ue_in;
            ut = ut_in;

        case 'velocity'
            % Velocity hold, speed control / gliding
            k = params.ctrl.velocity;
            e_u = k.u_ref - u;
            integ.u = clip(integ.u + k.Ki_u*e_u, -k.iLimit_u, k.iLimit_u);
            ut = clip(k.Kp_u*e_u + integ.u, 0, 1);

            e_theta = k.theta_ref - theta;
            integ.theta = clip(integ.theta + k.Ki*e_theta, -params.ctrl.iLimit_theta, params.ctrl.iLimit_theta);
            ue = k.Kp*e_theta - k.Kd*q + integ.theta;
            ua = -p + ua_in;
            ur = -r + ur_in;

        case 'cruise'
            % Altitude hold and velocity hold together
            k = params.ctrl.cruise;
            h = -z;
            e_u = k.u_ref - u;
            integ.u = clip(integ.u + k.Ki_u*e_u, -k.iLimit_u, k.iLimit_u);
            ut = clip(k.Kp_u*e_u + integ.u, 0, 1);

            h_err = clip(k.Kp_h*(k.h_ref - h) - k.Kd_h*(-w), -k.thetaMax, k.thetaMax);
            e_theta = h_err - theta;
            integ.theta = clip(integ.theta + k.Ki*e_theta, -params.ctrl.iLimit_theta, params.ctrl.iLimit_theta);
            ue = k.Kp*e_theta - k.Kd*q + integ.theta;
            ua = -p + ua_in;
            ur = -r + ur_in;

        otherwise
            error('mainAirplane:mode','unknown autopilot mode "%s"', params.ctrl.mode);
    end

    % Correct for actuator direction, then saturate at the physical surface stops
    delta.aileron  = clip(-ua, -params.act.aileronMax, params.act.aileronMax);
    delta.elevator = clip(-ue, -params.act.elevatorMax, params.act.elevatorMax);
    delta.rudder   = clip(-ur, -params.act.rudderMax, params.act.rudderMax);
    delta.throttle = clip(ut, 0, 1);
end

function [x0, pilot] = initialState(name, AltitudeMSL, trimSpeed, params)
    % The three conditions from flightSim_createAircraft.m, plus a solved trim.
    switch name
        case 'trimmed'
            [x0, pilot] = trimLevelFlight(trimSpeed, AltitudeMSL, params);

        case 'doNothing'
            x0 = [0; 0; -AltitudeMSL; trimSpeed; 0; 0; 0; 0; 0; 0; 0; 0];
            pilot = struct('aileron',  params.pilot.aileron,  ...
                           'elevator', params.pilot.elevator, ...
                           'rudder',   params.pilot.rudder,   ...
                           'throttle', params.pilot.throttle);
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
            error('mainAirplane:initialCondition','unknown initial condition "%s"', name);
    end
end

function [x0, u0] = trimLevelFlight(V, h, params)
    % Solve for straight and level flight at a given speed and altitude.
    % Three unknowns, angle of attack, elevator and throttle, against three residuals
    x_init = [0.0; 0.0; 0.5]; % zero alpha, zero elevator, half throttle
    step = 1e-6;

    for iteration = 1:100
        f = evalDynamics(x_init, V, h, params);
        if norm(f) < 1e-9
            break
        end
        df = zeros(3,3); % rate of change of f
        for k = 1:3
            nudged = x_init; % reset to the current guess value
            nudged(k) = nudged(k) + step;
            df(:,k) = (evalDynamics(nudged, V, h, params) - f)/step;
        end
        x_init = x_init - df\f;
    end

    alpha = x_init(1);
    x0 = [0; 0; -h; V*cos(alpha); 0; V*sin(alpha); 0; alpha; 0; 0; 0; 0];
    u0 = struct('aileron',0,'elevator',x_init(2),'rudder',0,'throttle',x_init(3));
end

function residual = evalDynamics(guess, V, h, params)
    % Accelerations left over at a candidate trim, for trimLevelFlight to drive to zero.
    alpha = guess(1);
    x = [0; 0; -h; V*cos(alpha); 0; V*sin(alpha); 0; alpha; 0; 0; 0; 0];
    delta = struct('aileron',0,'elevator',guess(2),'rudder',0,'throttle',guess(3));

    x_dot = dynamics(x, delta, params);

    residual = [x_dot(4); x_dot(6); x_dot(11)]; % u_dot, w_dot, q_dot
end

%% Numerical Linearization: xdot = A x + B u (central differences)
function [A, B] = linearizeDynamics(x0, control, params)
    nStates = length(x0);
    nInputs = length(fieldnames(control));
    A = zeros(nStates, nStates);
    B = zeros(nStates, nInputs);
    h = 1e-6;
    for k = 1:nStates
        dx = zeros(nStates, 1);
        dx(k) = h;
        A(:, k) = (dynamics(x0 + dx, control, params) - dynamics(x0 - dx, control, params)) / (2 * h);
    end
    eachControl = fieldnames(control);
    for k = 1:nInputs
        control_plus = control;
        control_minus = control;
        control_plus.(eachControl{k})  = control_plus.(eachControl{k}) + h;
        control_minus.(eachControl{k}) = control_minus.(eachControl{k}) - h;
        B(:, k) = (dynamics(x0, control_plus, params) - dynamics(x0, control_minus, params)) / (2 * h);
    end
    
    % disp('A ='); disp(A);
    % disp('B ='); disp(B);
    % disp('Open-loop eigenvalues of A:'); disp(eig(A));
end