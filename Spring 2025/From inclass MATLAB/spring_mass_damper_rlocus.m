clear; clc; close all
m = 1; % mass
k = 1; % spring stiffness
c = 1; % damper
A = [0 1; -k/m -c/m];
B = [0; 1/m];
C1 = [1 0]; % position output
C2 = [0 1]; % velocity output
D = 0;

%% assuming that we don't know the accurate values for m,k,c 
m_true = 1.05*m; % a scale that is a bit off (5% off)
k_true = 0.95*k; % also 5% off
c_true = 1.2*c; % 20% off
A_true = [0 1; -k_true/m_true -c_true/m_true];
B_true = [0; 1/m_true];


s = tf('s');
tf_ss_pos = C1*inv(s*eye(2) - A)*B; % transfer function from a state-space with position output
tf_ss_vel = C2*inv(s*eye(2) - A)*B; % transfer function from a state-space with velocity output

figure
rlocus(tf_ss_pos) % plot a root locus for position output
title("root locus position output")

figure
rlocus(tf_ss_vel) % plot a root locus for velocity output
title("root locus velocity output")

K = 2.37; % randomly pick a gain on the root locus
tf_cl_pos = minreal(K*tf_ss_pos/(1+K*tf_ss_pos)); % verify the location of the closed-loop poles
pole(tf_cl_pos)

t0 = 0;
dt = 0.01;
Tf = 20;
T = t0:dt:Tf;
x = zeros(2,length(T));
x0 = [0;0]; % initial state
x(:,1) = x0;
k = 2; % control gain
for t = 1:(length(T)-1)
    r = 1; % reference
    y = x(1,t); % feedback measurement
    e = r - y; % error
    u = k*e; % feedback controller

    % x_dot = A_true*x(:,t) + B_true*u; % off-design model
    x_dot = A*x(:,t) + B*u;
    x(:,t+1) = x(:,t) + x_dot*dt;
end
figure
plot(T,x(1,:),'-r',T,x(2,:),'-b')
legend("pos (m)", "vel (m/s)")
xlabel("time (s)")
ylabel("states")
grid on; grid minor