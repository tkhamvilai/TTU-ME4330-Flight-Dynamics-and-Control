clc; clear; close all
%% Airplane simulation: dynamics, aerodynamics, autopilot, and plots in one file.
%
% de Havilland DHC-2 Beaver. Geometry, mass properties and every aerodynamic coefficient
% are lifted from flightSim_createAircraft.m, and the autopilot modes are the example
% controllers from flightController.m. SI units throughout (m, kg, N, s). Local frame is
% NED, so positive z is down and an altitude of 2202 m is XD = -2202.
%
% States (12). Velocity is in the BODY frame, attitude is Euler angles, same layout as
% mainQuadrotor.m:
%    1:3    pos   position in the local NED frame, m
%    4:6    vel   velocity in the body frame, m/s   (u, v, w)
%    7:9    eul   roll, pitch, yaw, rad
%   10:12   rate  body angular rate, rad/s          (p, q, r)
%
% The control is the four pilot channels a real autopilot drives: aileron, elevator and
% rudder deflections in rad, and throttle from 0 to 1. flightController.m works in an
% internal sign convention where the surface command is the negative of the loop output
% (its "correct for actuator direction" step), and that convention is kept here so its
% gains carry over unchanged.
%
% Unlike flightSim_createAircraft.m this file needs no Aerospace Toolbox: the coefficient
% build-up, the standard atmosphere and the propeller curve are all written out below.

%% Parameters
% Everything the aircraft and the autopilot need, grouped by what it describes.
% Nothing below this section should contain a tuning number.

% ---------------------------------- Environment ----------------------------------
params.env.g = [0; 0; 9.80665]; % gravity, m/s^2
params.env.rho0 = 1.225; % sea level density, kg/m^3
params.env.T0 = 288.15; % sea level temperature, K
params.env.lapse = 0.0065; % troposphere lapse rate, K/m
params.env.R = 287.05; % specific gas constant, J/(kg*K)

% ------------------------------------- Body --------------------------------------
params.body.m = 2288.231; % mass, kg
params.body.J = [ 5788.0     0.0  -117.6;      % inertia, kg*m^2. Not diagonal: the Ixz
                     0.0  6928.9     0.0;      % term couples roll and yaw.
                  -117.6     0.0 11578.3];

% --------------------------------- Reference geometry ----------------------------
params.geom.S = 23.2300; % reference area, m^2
params.geom.b = 14.6300; % reference span, m
params.geom.c = 1.5875;  % reference chord, m

% ------------------------------ Aerodynamic coefficients -------------------------
% Body-axis build-up, exactly the values active in flightSim_createAircraft.m. The terms
% that file zeroed out are left at zero here and their original Beaver values are noted,
% so switching one back on is a one-character edit.
%
% Non-dimensional rates follow that file's convention: pb2V = p*b/(2V), qcV = q*c/V and
% rb2V = r*b/(2V).
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
params.aero.Cm_qcV  =  0;       % Beaver value -15.5600. Zeroed, so pitch damping comes
                                % only through CZ_qcV; see the note in the header.
params.aero.Cm_rb2V = -0.3118;

params.aero.Cn_0    =  0;       % Beaver value -0.0031
params.aero.Cn_b    =  0.0067;
params.aero.Cn_pb2V = -0.1585;
params.aero.Cn_rb2V = -0.1112;

% Control surface effectiveness
params.aero.CZ_de = -0.3980;
params.aero.Cm_de = -1.9210;

params.aero.Cl_da = -0.0992;
params.aero.Cn_da = -0.0039;
params.aero.CY_da = -0.0296;

params.aero.Cl_dr =  0.0069;
params.aero.Cn_dr = -0.0827;
params.aero.CY_dr =  0.1158;

% -------------------------------- Propulsion -------------------------------------
% The propeller curve from flightSim_createAircraft.m: a direct body-X force in newtons,
% not a coefficient, so it is never multiplied by dynamic pressure.
params.prop.throttle = (1/6)*(3 + (-3:3)); % breakpoints, 0 to 1
params.prop.thrust = 5000*(1 + tanh(-3:3)); % N, about 25 N at idle and 9975 N at full

% ------------------------------ Actuator limits ----------------------------------
params.act.aileronMax  = deg2rad(30);
params.act.elevatorMax = deg2rad(30);
params.act.rudderMax   = deg2rad(60);

% -------------------------------- Autopilot --------------------------------------
% Pick one mode. Every example from flightController.m is here; each keeps that file's
% structure and gains, including its integrators, which accumulate once per step rather
% than per second, so changing dt retunes those loops.
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
initialCondition = 'doNothing';
% initialCondition = 'trimmed';
trimSpeed = 55; % m/s, used by 'trimmed' only

[x0, params.pilot] = initialState(initialCondition, AltitudeMSL, trimSpeed, params);

x_hist = zeros(length(tspan),N_states);
x_dot_hist = zeros(length(tspan),N_states);
delta_hist = zeros(length(tspan),4); % aileron, elevator, rudder, throttle
air_hist = zeros(length(tspan),3); % V, alpha, beta
force_hist = zeros(length(tspan),3);
moment_hist = zeros(length(tspan),3);

%% Simulation
i = 1; % the number of iterations
x = x0; % initial state
integ.theta = 0; % pitch integrator
integ.u = 0; % airspeed integrator
for t = tspan
    % Air data, needed by both the controller and the aerodynamics
    air = airData(x, params);

    % Autopilot
    [delta, integ] = flightController(x, air, integ, params);

    % Equations of motion
    [x_dot, F, M] = dynamics(x, delta, air, params);

    % save results for plotting
    x_hist(i,:) = x';
    x_dot_hist(i,:) = x_dot';
    delta_hist(i,:) = [delta.aileron, delta.elevator, delta.rudder, delta.throttle];
    air_hist(i,:) = [air.V, air.alpha, air.beta];
    force_hist(i,:) = F';
    moment_hist(i,:) = M';

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
    h = -x(3,1);

    air.V = max(norm(v), 1e-3); % floored so a standstill cannot divide by zero
    air.alpha = atan2(v(3), v(1));
    air.beta = asin(min(max(v(2)/air.V, -1), 1));
    air.rho = density(h, params);
    air.qbar = 0.5*air.rho*air.V^2;
end

function rho = density(h, params)
    % ISA troposphere. Density is recomputed every step rather than frozen at the starting
    % altitude, which matters once a climb mode moves the aircraft several thousand feet.
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

    % Non-dimensional rates, using the same normalisation as flightSim_createAircraft.m
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

    % Rolling and yawing moments scale on span, pitching on chord
    F = air.qbar*S*[CX; CY; CZ] + [propThrust(delta.throttle, params); 0; 0];
    M = air.qbar*S*[b*Cl; c*Cm; b*Cn];
end

function [x_dot, F, M] = dynamics(x, delta, air, params)
    % States. Position does not affect the derivatives except through air density, which
    % airData has already resolved.
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
    % The example autopilots from flightController.m. Each one is written in that file's
    % internal sign convention, where the surface command is the negative of the loop
    % output, so the gains transfer across unchanged.
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
            integ.theta = clamp(integ.theta + k.Ki*e_theta, params.ctrl.iLimit_theta);
            ue = k.Kp*e_theta - k.Kd*q + integ.theta;
            ua = -p + ua_in;
            ur = -r + ur_in;

        case 'altitude'
            % Altitude, climb and level out
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
            integ.u = clamp(integ.u + k.Ki_u*e_u, k.iLimit_u);
            ut = min(max(k.Kp_u*e_u + integ.u, 0), 1);

            e_theta = k.theta_ref - theta;
            integ.theta = clamp(integ.theta + k.Ki*e_theta, params.ctrl.iLimit_theta);
            ue = k.Kp*e_theta - k.Kd*q + integ.theta;
            ua = -p + ua_in;
            ur = -r + ur_in;

        case 'cruise'
            % Altitude hold and velocity hold together
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
            error('mainAirplane:mode','unknown autopilot mode "%s"', params.ctrl.mode);
    end

    % Correct for actuator direction, then saturate at the physical surface stops
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

function [x0, pilot] = trimLevelFlight(V, h, params)
    % Solve for straight and level flight at a given speed and altitude.
    %
    % Three unknowns, angle of attack, elevator and throttle, against three residuals:
    % no axial acceleration, no normal acceleration, no pitching moment. Level flight
    % means the flight path angle is zero, so pitch attitude equals angle of attack.
    % Newton with finite-difference derivatives, which needs no toolbox.
    guess = [0.05; -0.05; 0.5]; % alpha, elevator, throttle
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
    % Accelerations left over at a candidate trim, for trimLevelFlight to drive to zero.
    alpha = guess(1);
    x = [0; 0; -h; V*cos(alpha); 0; V*sin(alpha); 0; alpha; 0; 0; 0; 0];
    delta = struct('aileron',0,'elevator',guess(2),'rudder',0,'throttle',guess(3));

    air = airData(x, params);
    x_dot = dynamics(x, delta, air, params);

    residual = [x_dot(4); x_dot(6); x_dot(11)]; % u_dot, w_dot, q_dot
end

function out = clamp(value, limit)
    % Symmetric saturation, used for both integrator anti-windup and surface stops.
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
