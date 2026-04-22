clear; clc; close all
% simulate and control the unstable spiral mode

%% simulation time
t0 = 0; % initial time (s)
Tf = 5; % final time (s)
dt = 0.001; % timestep (s)
T = t0:dt:Tf; % time vector

%% Dutch-Roll Motion
A = [-0.3844   -0.9866
      0.7882   -1.1411];
B = [-0.0020   -0.0414
     -1.4212  -23.9293];

%% LQR Control Design
% we know that the control signal u = -K*x
% For a LQR technique, K = -R^(-1)*B'*P
% P is obtained from A'*P + P*A + Q - P*B*R^(-1)*B'*P= 0
% We need to choose Q and R.
% Larger Q will stabilize the system faster, but use more control/energy
% Larger R will use less control/energy, but will stabilize slower

% chose Q and R
Q = [10 0; 
     0 10];
R = [1 0;
     0 1];
% solve for P from A'*P + P*A + Q - P*B*R^(-1)*B'*P = 0.
% MATLAB has built-in function to help with this
[K_lqr, P] = lqr(A,B,Q,R);

% Compute K from -R^(-1)*B'*P
K = R^(-1)*B'*P;

%% Simulation
% initialization
x = zeros(2,length(T)); % time-series of state vector
x(:,1) = [0; 1]; % initial side-slip angle and yaw rate
u = zeros(2,length(T)); % time-series of control vector

% run
for i = 1:length(T)
    u(:,i) = -K*x(:,i);
    x_dot = A*x(:,i) + B*u(:,i);
    x(:,i+1) = x(:,i) + x_dot * dt;
end

%% Plot
subplot(2,2,1)
plot(T, x(1,1:length(T)), '-r'); hold on
xlabel('Time (s)'); 
ylabel('Side-slip angle (rad)')
grid on; grid minor

subplot(2,2,2)
plot(T, x(2,1:length(T)), '-r'); hold on
xlabel('Time (s)'); 
ylabel('Yaw rate (rad/s)')
grid on; grid minor

subplot(2,2,3)
plot(T, u(1,1:length(T)), '-r'); hold on
xlabel('Time (s)'); 
ylabel('aileron deflection (rad)')
grid on; grid minor

subplot(2,2,4)
plot(T, u(2,1:length(T)), '-r'); hold on
xlabel('Time (s)'); 
ylabel('rudder deflection (rad)')
grid on; grid minor