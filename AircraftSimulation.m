clear; clc; close all

%% Parameters
% mass and geometry
m = 1247.379; % mass, kg
J = diag([1420.89721,4067.45384,4786.03736]); % moment of inertia, kg*m^2
b = 10.18032; % wingspan, m
c = 1.73736; % wing mean aerodynamic chord, m
xR = (29.5 - 25)/100; % CG location from CP as a % of MAC
S = 17.0942; % wing area, m^2
AR = b^2/S; % wing aspect ratio
e = 0.98; % efficiency factor
g = 9.80665; % gravity, m/s^2

% stability and control derivatives
CD_0 = 0.05;
CD_alpha = 0.33;
CD_Mach = 0.0;
CL_0 = 0.41;
CL_alpha = 4.44;
CL_alpha_dot = 0.0;
CL_q = 3.8;
CL_Mach = 0.0;
CL_de = -0.923;
Cm_alpha = -0.683;
Cm_alpha_dot = -4.36;
Cm_q = -9.96;
Cm_Mach = 0.0;
Cm_de = -0.923;
Cy_beta = -0.564;
Cy_dr = 0.157;
Cl_beta = -0.074;
Cl_p = -0.41;
Cl_r = 0.107;
Cl_da = -0.134;
Cl_dr = 0.107;
Cn_beta = -0.071;
Cn_p = -0.0575;
Cn_r = -0.125;
Cn_da = -0.0035;
Cn_dr = -0.072;

Cx_dpt = 0.1161;
Cz_dpt = -0.1563;
Cm_dpt = -0.079;

%% Initial states
pos = [0;0;-1500]; % m
vel = [54.194;0;0]; % m/s
att = [0;0;0]; % rad;
rate = [0;0;0]; % rad/s
wind_vel = [0;0;0]; % m/s
alpha_dot = 0;
da = 0; % rad
de = 0; % rad
dr = 0; % rad

%% Simulation Setup
Tf = 500; % sec
dt = 0.01;
tspan = 0:dt:Tf;
itr = 1;

state = zeros(12,length(tspan));
state_dot = zeros(12,length(tspan));
controls = zeros(6,length(tspan));

for t = tspan
 %% Atmospheric Model
    height = -pos(3);
    [Temp,a,Press,rho,nu,mu] = atmosisa(height); % temperature, speed of sound, pressure, density, kinematic viscosity, dynamic viscosity
 
 %% Wind Frame
    u = vel(1) - wind_vel(1); 
    v = vel(2) - wind_vel(2); 
    w = vel(3) - wind_vel(3);
    V = sqrt(u^2 + v^2 + w^2); % free-stream velocity
    alpha = atan2(w,u); % angle of attack
    beta = asin(v/V); % side-slip angle
    Mach = V/a; % mach number
    Q = 1/2*rho*V^2; % dynamic pressure

 %% Engine Model
    n = 1485; % rpm (full at 2300)
    Pz = 20; % mainfold pressure ["Hg] 35mps
    P = 0.7355*(-326.5+(0.00412*(Pz+7.4)*(n+2010)+(408-0.0965*n)*(1-rho/1.225))); % engine power
    dpt = 0.08696+191.18*(P*2/rho/V^3);

 %% Aerodynamics Model
    p = rate(1);
    q = rate(2);
    r = rate(3);

    CL = CL_0 + CL_alpha*alpha + CL_alpha_dot*alpha_dot + CL_q*q*c/2/V + CL_Mach*Mach + CL_de*de;
    CL = CL/sqrt(abs(1-Mach^2));
    CD_i = CL^2/pi/e/AR; % induce drag coefficient
    CD = CD_0 + CD_i + CD_alpha*alpha + CD_Mach*Mach;
    Cy = Cy_beta*beta + Cy_dr*dr;
    
    CDCL = [CD; CL];
    A = [-cos(alpha)*cos(beta) -sin(alpha)*cos(beta); sin(alpha) -cos(alpha)];
    B = [-sin(beta); 0];
    CxCz = A\(CDCL - B);
    Cx = CxCz(1) + Cx_dpt*dpt;
    Cz = CxCz(2) + Cz_dpt*dpt;
    
    Cl = Cl_beta*beta + Cl_da*da + Cl_dr*dr + b/2/V*Cl_r*r;
    Cm = Cm_alpha*alpha + Cm_Mach*Mach + Cm_de*de + c/2/V*(Cm_q*q + Cm_alpha_dot*alpha_dot) + xR*CL + Cm_dpt*dpt;
    Cn = Cn_beta*beta + Cn_da*da + Cn_dr*dr + b/2/V*(Cn_p*p + Cn_r*r);
    
    X = Q*S*Cx;
    Y = Q*S*Cy;
    Z = Q*S*Cz;
    L = Q*S*b*Cl;
    M = Q*S*c*Cm;
    N = Q*S*b*Cn;

    Forces = [X;Y;Z];
    Moments = [L;M;N];

 %% Rigid-Body Dynamics   
    phi = clip(att(1),-pi,pi); % roll
    theta = clip(att(1),-pi/2,pi/2); % pitch
    psi = clip(att(3),-pi,pi); % yaw
    
    C_b2i = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
             cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
             -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)]; % body to inertia
    C_euler = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
               0 cos(phi) -sin(phi);
               0 sin(phi)/cos(theta) cos(phi)/cos(theta)]; % for euler kinematics
    
    pos_dot = C_b2i*vel;
    vel_dot = -cross(rate,vel) + Forces/m + C_b2i'*[0;0;g];
    att_dot = C_euler*rate;
    rate_dot = J\(Moments-cross(rate,J*rate));

 %% Numerical Integration
    pos = pos + pos_dot*dt;
    vel = vel + vel_dot*dt;
    att = wrapToPi(att + att_dot*dt);
    rate = rate + rate_dot*dt;
    
    alpha_dot = (u*vel_dot(3) - w*vel_dot(1))/(u^2 + w^2); % constant wind velocity

 %% Save Data for plotting
    state(:,itr) = [pos;vel;att;rate];
    state_dot(:,itr) = [pos_dot;vel_dot;att_dot;rate_dot];
    controls(:,itr) = [Forces;Moments];
    itr = itr + 1;
end

%% Plots
figure(1)
subplot(12,1,1)
plot(tspan, state(1,:), 'k', 'LineWidth', 1.5)
title('x'); xlabel('time'); grid on
subplot(12,1,2)
plot(tspan, state(2,:), 'k', 'LineWidth', 1.5)
title('y'); xlabel('time'); grid on
subplot(12,1,3)
plot(tspan, -state(3,:), 'k', 'LineWidth', 1.5)
title('h'); xlabel('time'); grid on

subplot(12,1,4)
plot(tspan, state(4,:), 'k', 'LineWidth', 1.5)
title('u'); xlabel('time'); grid on
subplot(12,1,5)
plot(tspan, state(5,:), 'k', 'LineWidth', 1.5)
title('v'); xlabel('time'); grid on
subplot(12,1,6)
plot(tspan, state(6,:), 'k', 'LineWidth', 1.5)
title('w'); xlabel('time'); grid on

subplot(12,1,7)
plot(tspan, state(7,:).*180/pi, 'k', 'LineWidth', 1.5)
title('phi (degree)'); xlabel('time'); grid on
subplot(12,1,8)
plot(tspan, state(8,:).*180/pi, 'k', 'LineWidth', 1.5)
title('theta (degree)'); xlabel('time'); grid on
subplot(12,1,9)
plot(tspan, state(9,:).*180/pi, 'k', 'LineWidth', 1.5)
title('psi (degree)'); xlabel('time'); grid on

subplot(12,1,10)
plot(tspan, state(10,:), 'k', 'LineWidth', 1.5)
title('p'); xlabel('time'); grid on
subplot(12,1,11)
plot(tspan, state(11,:), 'k', 'LineWidth', 1.5)
title('q'); xlabel('time'); grid on
subplot(12,1,12)
plot(tspan, state(12,:), 'k', 'LineWidth', 1.5)
title('r'); xlabel('time'); grid on

figure(2)
subplot(6,1,1)
plot(tspan, controls(1,:), 'k', 'LineWidth', 1.5)
title('X'); xlabel('time'); grid on
subplot(6,1,2)
plot(tspan, controls(2,:), 'k', 'LineWidth', 1.5)
title('Y'); xlabel('time'); grid on
subplot(6,1,3)
plot(tspan, controls(3,:), 'k', 'LineWidth', 1.5)
title('Z'); xlabel('time'); grid on
subplot(6,1,4)
plot(tspan, controls(4,:), 'k', 'LineWidth', 1.5)
title('L'); xlabel('time'); grid on
subplot(6,1,5)
plot(tspan, controls(5,:), 'k', 'LineWidth', 1.5)
title('M'); xlabel('time'); grid on
subplot(6,1,6)
plot(tspan, controls(6,:), 'k', 'LineWidth', 1.5)
title('N'); xlabel('time'); grid on