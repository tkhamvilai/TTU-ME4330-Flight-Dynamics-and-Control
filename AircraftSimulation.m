clear; clc; close all
AircraftParameters
AircraftInitialization

%% Simulation Setup
Tf = 500; % sec
dt = 0.01;
tspan = 0:dt:Tf;
itr = 1;

state = zeros(12,length(tspan));
state_dot = zeros(12,length(tspan));
surfaces = zeros(9,length(tspan));
controls = zeros(6,length(tspan));
wind_terms = zeros(3,length(tspan));

u_rpm = 2500; % rpm setpoint

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
    ue = deg2rad(2.5); % pull
    u_rpm = 2700; % full throttle
    
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

    % Altitude
    % h_ref = 1800;
    % h_err = clip(0.01*(h_ref - height) - 0.01*w, -deg2rad(15), deg2rad(15));
    % Kp_theta = 1;
    % Ki_theta = 0.01;
    % Kd_theta = 2;
    % e_theta = h_err - theta;
    % eI_theta = eI_theta + Ki_theta*e_theta;
    % eI_theta = clip(eI_theta,-1,1);
    % ue = Kp_theta*e_theta - Kd_theta*q + eI_theta;

    % Roll-Yaw SAS
    % Kp = 0.75; % rate roll gain
    % if t > 10 && t < 40 % reference roll command
    %     rp = deg2rad(3);
    % elseif t > 50 && t < 80
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

    % Velocity Hold
    % u_ref = 55; % m/s
    % e_u = u_ref - u;
    % eI_u = eI_u + e_u;
    % eI_u = clip(eI_u,-500,500);
    % u_rpm = clip(2500 + 2*e_u + eI_u, idle_rpm, max_rpm);
    %
    % Kp_theta = 1;
    % Ki_theta = 0.01;
    % Kd_theta = 1;
    % e_theta = deg2rad(0) - theta;
    % eI_theta = eI_theta + Ki_theta*e_theta;
    % eI_theta = clip(eI_theta,-1,1);
    % ue = Kp_theta*e_theta - Kd_theta*q + eI_theta;

 %% Actuator Dynamics
    % with actuator dynamics
    da_dot = (-ua - da)/0.1; % negative gain in tf, take care off +da to -L
    de_dot = (-ue - de)/0.1; % negative gain in tf, take care off +de to -M
    dr_dot = (-ur - dr)/0.1; % negative gain in tf, take care off +dr to -N
    n_rpm_dot = (u_rpm - n_rpm)/0.1;
    
    % without actuator dynamics
    % da = -ua;
    % de = -ue;
    % dr = -ur;
    % n_rpm = u_rpm;

 %% Engine Model
    Pz = 20; % mainfold pressure ["Hg] 35mps
    P = 0.7355*(-326.5+(0.00412*(Pz+7.4)*(n_rpm+2010)+(408-0.0965*n_rpm)*(1-rho/1.225))); % engine power
    if P < 0
        P = 0;
        dpt = 0;
    else
        dpt = 0.08696+191.18*(P*2/rho/V^3);
    end

 %% Aerodynamics Model
    CL = CL_0 + CL_alpha*alpha + CL_alpha_dot*alpha_dot + CL_q*q*c/2/V + CL_Mach*Mach + CL_de*de;
    CL = CL/sqrt(abs(1-Mach^2));
    CD_i = CL^2/pi/e/AR; % induce drag coefficient
    CD = CD_0 + CD_i + CD_alpha*alpha + CD_Mach*Mach;
    Cy = Cy_beta*beta + Cy_dr*dr;
    
    CDCL = [CD; CL];
    A = [-cos(alpha)*cos(beta) -sin(alpha)*cos(beta); sin(alpha) -cos(alpha)];
    B = [-sin(beta); 0];
    CxCz = A\(CDCL - B*Cy);
    Cx = CxCz(1) + Cx_dpt*dpt;
    Cz = CxCz(2) + Cz_dpt*dpt;
    
    Cl = Cl_beta*beta + Cl_da*da + Cl_dr*dr + b/2/V*Cl_r*r;
    Cm = Cm_alpha*alpha + Cm_Mach*Mach + Cm_de*de + c/2/V*(Cm_q*q + 0*Cm_alpha_dot*alpha_dot) + xR*CL + Cm_dpt*dpt;
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
    n_rpm = n_rpm + n_rpm_dot*dt;

    % rw_interal = rw_interal + rw_interal_dot*dt;

 %% Save Data for plotting
    state(:,itr) = [pos;vel;att;rate];
    state_dot(:,itr) = [pos_dot;vel_dot;att_dot;rate_dot];
    surfaces(:,itr) = [ua;ue;ur;u_rpm;da;de;dr;dpt;n_rpm];
    controls(:,itr) = [Forces;Moments];
    wind_terms(:,itr) = [V;rad2deg(alpha);rad2deg(beta)];
    itr = itr + 1;
end

%% Plots
figure(1)
subplot(4,1,1)
plot(tspan,state(1,:),'-r',tspan,state(2,:),'-g',tspan,-state(3,:),'-b');
legend('x','y','h')
xlabel('time (s)')
ylabel('position (m)')
grid on; grid minor

subplot(4,1,2)
plot(tspan,state(4,:),'-r',tspan,state(5,:),'-g',tspan,state(6,:),'-b');
legend('u','v','w')
xlabel('time (s)')
ylabel('velocity (m/s)')
grid on; grid minor

subplot(4,1,3)
plot(tspan,state(7,:)*180/pi,'-r',tspan,state(8,:)*180/pi,'-g',tspan,state(9,:)*180/pi,'-b');
legend('phi','theta','psi')
xlabel('time (s)')
ylabel('Euler angles (deg)')
grid on; grid minor

subplot(4,1,4)
plot(tspan,state(10,:)*180/pi,'-r',tspan,state(11,:)*180/pi,'-g',tspan,state(12,:)*180/pi,'-b');
legend('p','q','r')
xlabel('time (s)')
ylabel('angular velocity (deg/s)')
grid on; grid minor

figure(2)
subplot(2,1,1)
plot(tspan,controls(1,:),'-r',tspan,controls(2,:),'-g',tspan,controls(3,:),'-b');
legend('X','Y','Z')
xlabel('time (s)'); 
ylabel('Forces (N)')
grid on; grid minor

subplot(2,1,2)
plot(tspan,controls(4,:),'-r',tspan,controls(5,:),'-g',tspan,controls(6,:),'-b');
legend('L','M','N')
xlabel('time (s)'); 
ylabel('Moments (Nm)')
grid on; grid minor

figure(3)
subplot(4,1,1)
plot(tspan,surfaces(1,:),'-r',tspan,surfaces(2,:),'-g',tspan,surfaces(3,:),'-b');
legend('ua','ue','ur')
xlabel('time (s)'); 
ylabel('Control Commands')
grid on; grid minor

subplot(4,1,2)
plot(tspan,surfaces(5,:)*180/pi,'-r',tspan,surfaces(6,:)*180/pi,'-g',tspan,surfaces(7,:)*180/pi,'-b');
legend('da','de','dr')
xlabel('time (s)'); 
ylabel('Surface Deflection (deg)')
grid on; grid minor

subplot(4,1,3)
plot(tspan,surfaces(4,:),'-r',tspan,surfaces(9,:),'-g');
legend('u_{rpm}','n_{rpm}')
xlabel('time (s)'); 
ylabel('RPM')
grid on; grid minor

subplot(4,1,4)
plot(tspan,surfaces(8,:),'-r');
legend('dpt')
xlabel('time (s)'); 
ylabel('throttle setting')
grid on; grid minor

figure(5)
visualization(state(1,:),-state(2,:),-state(3,:),-state(8,:),state(7,:),-state(9,:),0.01,3000,'cessna');