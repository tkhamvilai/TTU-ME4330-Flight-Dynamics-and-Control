clear; clc; close all
AircraftParameters
AircraftInitialization

%% Simulation Setup
Tf = 100; % sec
dt = 0.01;
tspan = 0:dt:Tf;
itr = 1;

state = zeros(12,length(tspan));
state_dot = zeros(12,length(tspan));
surfaces = zeros(6,length(tspan));
controls = zeros(6,length(tspan));
wind_terms = zeros(3,length(tspan));

for t = tspan
 %% Atmospheric Model
    height = -pos(3);
    [Temp,a,Press,rho,nu,mu] = atmosisa(height); % temperature, speed of sound, pressure, density, kinematic viscosity, dynamic viscosity
 
 %% Feedback States
    x = pos(1);
    y = pos(2);
    z = pos(3);
    u = vel(1) - wind_vel(1); 
    v = vel(2) - wind_vel(2); 
    w = vel(3) - wind_vel(3);
    phi = clip(att(1),-pi,pi); % roll
    theta = clip(att(2),-pi/2,pi/2); % pitch
    psi = clip(att(3),-pi,pi); % yaw
    p = rate(1);
    q = rate(2);
    r = rate(3);

 %% Wind Frame
    V = sqrt(u^2 + v^2 + w^2); % free-stream velocity
    alpha = atan2(w,u); % angle of attack
    beta = asin(v/V); % side-slip angle
    Mach = V/a; % mach number
    Q = 1/2*rho*V^2; % dynamic pressure

 %% Controls
    % Free-Fall
    ua = 0;
    ue = 0;
    ur = 0;

    % Climb
    % ue = deg2rad(2.5);
    
    % Pitch SAS
    % ue = -q;

    % Pitch CAS (PID)
    % Kp_theta = 1;
    % Ki_theta = 0.01;
    % Kd_theta = 1;
    % e_theta = deg2rad(15) - theta;
    % eI_theta = eI_theta + Ki_theta*e_theta;
    % eI_theta = clip(eI_theta,-1,1);
    % ue = Kp_theta*e_theta - Kd_theta*q + eI_theta;

    % Roll-Yaw SAS
    % Kp = 1; % rate roll gain
    % if t > 1 && t < 30 % reference roll command
    %     rp = deg2rad(3);
    % elseif t > 40 && t < 70
    %     rp = deg2rad(-3);
    % else
    %     rp = 0;
    % end
    % ua = rp - Kp*p;
    % 
    % rr = 0; % reference yaw command
    % Kr = 1; % rate yaw gain
    % % washout filter
    % tau_w = 1;
    % rw_interal_dot = (r - rw_interal)/tau_w;
    % rw = r - rw_interal;
    % ur = rr - Kr*rw;

    % Bank-Heading CAS (LQR Full-State Feddback)
    % phi_ref = deg2rad(5);
    % psi_ref = deg2rad(10);
    % K = [-0.2837   -0.6723   -0.4814   -0.9804   -0.7918
    %      -0.0974    0.4676   -0.8867    0.4932   -0.6108];
    % ua_ur = -K*[0-beta; 0-p; 0-r; phi_ref - phi; psi_ref - psi];
    % ua = ua_ur(1);
    % ur = ua_ur(2);

 %% Actuator Dynamics
    % with actuator dynamics
    da_dot = (-ua - da)/0.1; % negative gain in tf, take care off +da to -L
    de_dot = (-ue - de)/0.1; % negative gain in tf, take care off +de to -M
    dr_dot = (-ur - dr)/0.1; % negative gain in tf, take care off +dr to -N
    
    % without actuator dynamics
    % da = -ua;
    % de = -ue;
    % dr = -ur;

 %% Engine Model
    n = 2500; % rpm
    Pz = 20; % mainfold pressure ["Hg] 35mps
    P = 0.7355*(-326.5+(0.00412*(Pz+7.4)*(n+2010)+(408-0.0965*n)*(1-rho/1.225))); % engine power
    dpt = 0.08696+191.18*(P*2/rho/V^3);

 %% Aerodynamics Model
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

    da = da + da_dot*dt;
    de = de + de_dot*dt;
    dr = dr + dr_dot*dt;

    % rw_interal = rw_interal + rw_interal_dot*dt;

 %% Save Data for plotting
    state(:,itr) = [pos;vel;att;rate];
    state_dot(:,itr) = [pos_dot;vel_dot;att_dot;rate_dot];
    surfaces(:,itr) = [ua;ue;ur;da;de;dr];
    controls(:,itr) = [Forces;Moments];
    wind_terms(:,itr) = [V;rad2deg(alpha);rad2deg(beta)];
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

figure(3)
visualization(state(1,:),-state(2,:),-state(3,:),-state(8,:),state(7,:),-state(9,:),0.1,100,'cessna');