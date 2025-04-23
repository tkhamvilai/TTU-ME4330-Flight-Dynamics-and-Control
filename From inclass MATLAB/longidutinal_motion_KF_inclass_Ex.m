clear; 
% clc; 
close all
Xu = -0.045;
Zu = -0.369;
Mu = 0;
Xw = 0.036;
Zw = -2.02;
Mw = -0.05;
Xwdot = 0;
Zwdot = 0;
Mwdot = -0.0051;
u0 = 176; % ft/s
Za = u0*Zw;
Ma = u0*Mw;
Madot = u0*Mwdot;
Xq = 0;
Zq = 0;
Mq = -2.05;
g = 32.2; % not SI unit here

% This A_longidual is an actual A matrix (ground truth model)
A_longidutal = [Xu Xw 0 -g; % delta u
                Zu Zw u0 0; % delta w
                Mu+Mwdot*Zu Mw+Mwdot*Zw Mq+Mwdot*u0 0; % delta q
                0 0 1 0]; % delta theta

% A here is an estimated model, which is off by somewhere in between of +-10%
A = A_longidutal .* (rand(4,4)*(1.1-0.9)+0.9);
A(1,4) = -g; % we know the gravity pretty well
A(4,3) = 1; % delta theta dot = delta q
 
% C here is a measurement model, meaning that we can only measure 2 out of 4 states
C = [1 0 0 0; % measure delta u (forward velocity from a pitot tube)
     0 0 1 0];% measure delta q (pitch rate from a gyroscope)

% The goal here is to estimate delta w and delta theta with Kalman filter
% Note that some control design technique requires full state feedback, e.g., all 4 states in this example

%% Control Design
% you can make up these numbers
Xde = 0;
Xdt = 0.1161;
Zde = -0.1615;
Zdt = -0.1563;
Mde = -12.1234;
Mdt = -0.0790;
B = [Xde Xdt;
     Zde Zdt;
     Mde+Mwdot*Zde Mdt+Mwdot*Zdt;
     0 0];

%% Simulation and State Estimation
t0 = 0; % initial time
dt = 0.01; % time step
Tf = 200; % final time step
T = t0:dt:Tf; % time vector
x = zeros(4,length(T)); % truth state vector
x0 = [5;1;0;pi/6]; % initial truth state difference
x(:,1) = x0; % set the initial truth state

y = zeros(2,length(T)); % measurement vector, we can only measure 2 states

% estimator
x_est = zeros(4,length(T)); % estimate state vector, initially zero 4x1 vector
P_est = zeros(4,4,length(T)); % estimate state error covariance, 4x4 matrix
P_est(:,:,1) = eye(4); % initial error covariance is an identity
Q = eye(4,4); % a weight matrix indicating how much we trust our model
R = eye(2,2); % a weight matrix indicating how much we trust our measurement

% LQR control technique
K_lqr = lqr(A,B,Q,R); % feedback control gain
CC = [1 0 0 0]; % we only put a reference for delta u, the reference for delta q needs to be zero; otherwise, our plane will keep pitching during cruise
BB = B(:,2); % only thrust immediately affects the change in forward speed, elevator deflection affects pitch rate, then pitch rate affects pitch angle and/or angle of attack, then forward speed
K_lqr_u = K_lqr(1,:);
Kr = -inv(CC*inv(A-BB*K_lqr_u)*BB); % reference signal gain

for t = 1:(length(T)-1)
    % controller
    r = [10;0]; % reference for delta u (10) and delta q (0)
    u = -K_lqr*x_est(:,t) + Kr*r(1); % LQR controller with estimated state feedback

    % simulate the ground truth
    x_dot = A_longidutal*x(:,t) + B*u; % A_longidutal is the ground truth
    x(:,t+1) = x(:,t) + dt*x_dot;

    % to simulate the sensor measurement
    y(:,t) = C*x(:,t) + randn(2,1); % take two states from the ground truth then add some noises

    % Kalman filter prediction step
    % A here is the estimation of A_longidutal
    x_est_dot = A*x_est(:,t) + B*u; % predict the state vector derivative
    x_est(:,t+1) = x_est(:,t) + dt*x_est_dot; % integrate it
    P_est_dot = A*P_est(:,:,t) + P_est(:,:,t)*A' + Q; % predict the state error covariance
    P_est(:,:,t+1) = P_est(:,:,t) + P_est_dot*dt; % integrate it

    % Kalman filter correction step
    % r is a residual, which is a difference between the sensor measurement and the predicted measurement
    r = y(:,t) - C*x_est(:,t+1); % residual
    S = C*P_est(:,:,t+1)*C' + R; % S is residual covariance
    K_kalman = P_est(:,:,t+1)*C'*inv(S); % K_kalman is a kalman gain
    x_est(:,t+1) = x_est(:,t+1) + K_kalman*r; % correct the state vector
    P_est(:,:,t+1) = (eye(4) - K_kalman*C)*P_est(:,:,t+1)*(eye(4) - K_kalman*C)' + K_kalman*R*K_kalman'; % correct the state error covariance
end

%% Plot
subplot(4,1,1)
plot(T,x_est(1,:),"-r",T,x(1,:),"--b")
legend("estimate delta u","true delta u")
xlabel("delta u")
ylabel("time (s)")
grid on; grid minor

subplot(4,1,2)
plot(T,x_est(2,:),"-r",T,x(2,:),"--b")
legend("estimate delta w","true delta u")
xlabel("delta w")
ylabel("time (s)")
grid on; grid minor

subplot(4,1,3)
plot(T,x_est(3,:),"-r",T,x(3,:),"--b")
legend("estimate delta q","true delta u")
xlabel("delta q")
ylabel("time (s)")
grid on; grid minor

subplot(4,1,4)
plot(T,x_est(4,:),"-r",T,x(4,:),"--b")
legend("estimate delta theta","true delta u")
xlabel("delta theta")
ylabel("time (s)")
grid on; grid minor