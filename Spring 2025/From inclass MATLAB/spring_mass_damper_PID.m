clear; clc; close all
m = 1; % mass
kp = 1; % spring stiffness
c = 1; % damper
A = [0 1; -kp/m -c/m];
B = [0; 1/m];
C1 = [1 0]; % position output
C2 = [0 1]; % velocity output
D = 0;

%% assuming that we don't know the accurate values for m,k,c 
m_true = 1.05*m; % a scale that is a bit off (5% off)
k_true = 0.95*kp; % also 5% off
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
kp = 2; % control gain
ki = 1; % integral gain
kd = 0.5; % derivative gain
e = 0; % initial error
e_int = 0; % integration of the error
e_prev = 0; % previous error
for t = 1:(length(T)-1)
    r_pos = 2; % position reference
    % y_pos = x(1,t); % position feedback measurement
    y_pos = x(1,t) + randn; % position feedback measurement with a noise
    e = r_pos - y_pos; % position error

    r_vel = 0; % velocity reference
    y_vel = x(2,t) + randn; % velocity feedback measurement with a noise
    ed = r_vel - y_vel;

    % this naive derivative implementation amplifies a noise in your sensor measurement.
    % ed = (e - e_prev)/dt; % error derivative
    u = kp*e + ki*e_int + kd*ed; % PID feedback controller

    % x_dot = A_true*x(:,t) + B_true*u; % off-design model
    x_dot = A*x(:,t) + B*u;
    x(:,t+1) = x(:,t) + x_dot*dt;
    e_int = e_int + e*dt; % for ki
    e_prev = e; % for kd
end
figure
plot(T,x(1,:),'-r',T,x(2,:),'-b')
legend("pos (m)", "vel (m/s)")
xlabel("time (s)")
ylabel("states")
grid on; grid minor