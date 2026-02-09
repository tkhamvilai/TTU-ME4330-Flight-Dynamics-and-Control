clear; clc; close all

%% time-related parameters
% integrate numerically x_dot = t^2
t0 = 0; % initial time
tf = 50; % final time
dt = 0.01; % time step
tspan = t0:dt:tf; % time span

%% pre-allocation
% 3 row represents 3 dimension
% each column represents time step
pos = zeros(3,length(tspan));
vel = zeros(3,length(tspan));
att = zeros(3,length(tspan));
rate = zeros(3,length(tspan));

pos(:,1) = [0; 0; 000]; % initial altitude is 2000 m
vel(:,1) = [400; 0; 0]; % initial x velocity is 400 m/s
att(:,1) = [0; 0; 0]; % initial orientation
rate(:,1) = [0; 0; 0]; % initial angular velocity

pos_dot = zeros(3,length(tspan));
vel_dot = zeros(3,length(tspan));
att_dot = zeros(3,length(tspan));
rate_dot = zeros(3,length(tspan));

%% some aircraft parameters
m = 1; % mass, kg
J = 1*eye(3); % inertia matrix, kg-m^2
g = [0; 0; 9.81]; % gravity vector in inertial frame, m/s^2
Forces = [100; 0; -m*g(3)] .* ones(3,length(tspan)); % force in body frame, N
Moments = [0; 0; 0] .* ones(3,length(tspan)); % moment in body frame, N-m

%% simulation
itr = 1;
for t = tspan

    % if t > 10 && t <= 15
    %     Forces(1,itr) = 0;
    % elseif t > 15 && t <= 25
    %     Forces(1,itr) = 300;
    % elseif t > 25
    %     Forces(1,itr) = -5000;
    % end
    % Forces(1,itr) = -vel(1,itr) + 1000 -0*pos(1,itr);
    Moments(1,itr) = sin(t)*tan(t)*t^3; %deg2rad(10) + att(1,itr) -rate(1,itr);

    x = pos(1,itr);
    y = pos(2,itr);
    z = pos(3,itr);
    u = vel(1,itr);
    v = vel(2,itr);
    w = vel(3,itr);
    phi = att(1,itr); % roll
    theta = att(2,itr); % pitch
    psi = att(3,itr); % yaw
    p = rate(1,itr);
    q = rate(2,itr);
    r = rate(3,itr);

    C_phi   = [1     0        0     ;
               0  cos(phi) sin(phi) ;
               0 -sin(phi) cos(phi)];
    C_theta = [cos(theta)  0  -sin(theta) ;
                   0       1      0       ;
               sin(theta)  0   cos(theta)];
    C_psi   = [ cos(psi) sin(psi) 0;
               -sin(psi) cos(psi) 0;  
                   0        0     1];
    C_bI = C_phi * C_theta * C_psi; % rotation matrix from inertia to body
    C_euler = [1 sin(phi)*tan(theta)  cos(phi)*tan(theta) ;
               0 cos(phi)            -sin(phi)            ;
               0 sin(phi)/cos(theta)  cos(phi)/cos(theta)];

    pos_dot(:,itr) = C_bI'*vel(:,itr);
    vel_dot(:,itr) = -cross(rate(:,itr),vel(:,itr)) + Forces(:,itr)/m + C_bI*g;
    att_dot(:,itr) = C_euler * rate(:,itr);
    rate_dot(:,itr) = inv(J)*(Moments(:,itr) - cross(rate(:,itr), J*rate(:,itr)));

    if t < tspan(end) % do an integration if we are not at the final time
        pos(:,itr+1) = pos(:,itr)          + pos_dot(:,itr) * dt;
        vel(:,itr+1) = vel(:,itr)          + vel_dot(:,itr) * dt;
        att(:,itr+1) = wrapToPi(att(:,itr) + att_dot(:,itr) * dt);
        rate(:,itr+1) = rate(:,itr)        + rate_dot(:,itr) * dt;
    end
    itr = itr + 1; % increment the iterator
end

%% visualization
subplot(4,1,1) % we will plot 4 row and 1 column and start with the first one
plot(tspan,pos(1,:),'-r',tspan,pos(2,:),'-g',tspan,pos(3,:),'-b')
grid on; grid minor
xlabel('time (s)')
ylabel('position (m)')
title('position vs time')
legend('x','y','z')

subplot(4,1,2)
plot(tspan,vel(1,:),'-r',tspan,vel(2,:),'-g',tspan,vel(3,:),'-b')
grid on; grid minor
xlabel('time (s)')
ylabel('velocity (m/s)')
title('velocity vs time')
legend('u','v','w')

subplot(4,1,3)
plot(tspan,rad2deg(att(1,:)),'-r',tspan,rad2deg(att(2,:)),'-g',tspan,rad2deg(att(3,:)),'-b')
grid on; grid minor
xlabel('time (s)')
ylabel('attitude/orientation (deg)')
title('attitude vs time')
legend('\phi','\theta','\psi')

subplot(4,1,4)
plot(tspan,rate(1,:),'-r',tspan,rate(2,:),'-g',tspan,rate(3,:),'-b')
grid on; grid minor
xlabel('time (s)')
ylabel('angular velocity (rad/s)')
title('angular velocity vs time')
legend('p','q','r')

figure()
subplot(2,1,1)
plot(tspan,Forces(1,:),'-r',tspan,Forces(2,:),'-g',tspan,Forces(3,:),'-b')
grid on; grid minor
xlabel('time (s)')
ylabel('Forces (N)')
title('Forces vs time')
legend('X','Y','Z')

subplot(2,1,2)
plot(tspan,Moments(1,:),'-r',tspan,Moments(2,:),'-g',tspan,Moments(3,:),'-b')
grid on; grid minor
xlabel('time (s)')
ylabel('Moments (N-m)')
title('Moments vs time')
legend('L','M','N')