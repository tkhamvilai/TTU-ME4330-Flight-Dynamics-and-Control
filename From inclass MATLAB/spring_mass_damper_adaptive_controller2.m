clear; clc; close all
% Adaptive Control
% Matrix example

%% System parameters
% only use in the simulation but not in the control design
Lp = -0.5; % roll damping
Lda = 2; % roll moment coefficient due to aileron deflections

%% Roll angle instead of a roll rate
A = [0 1; 0 Lp];
B = [0; Lda];

%% Adaptive control
% below are design parameters (our choice)
gamma_x = eye(2); % an identity matrix because of two states
gamma_r = 1; % still have one control
Q = eye(2); % an identity matrix because of two states
% We will design a reference model properly by specifying desired 
% natural frequency and damping ratio
wn = 1; % desired natural frequency
c = 0.7; % desired damping ratio
A_ref = [0 1; -wn^2 -2*c*wn];
B_ref = [0; wn^2];

% this is required to be known from the system
% we know that +aileron deflection gives +roll moment
% so sign B is positive
signB = [0; 1];

% initial gains (also our choice)
kx = [0;0];
kr = 0;

% P matrix is a solve to A_ref'P + PA_ref + Q = 0 (Lyapunov equation)
P = lyap(A_ref',Q);

%% Simulation
t0 = 0;
dt = 0.01;
Tf = 25;
T = t0:dt:Tf;
x = zeros(2,length(T));
x0 = 0.5; % initial roll rate
x(:,1) = x0;

x_ref = zeros(2,length(T)); % reference model, note that init x_ref(:,1) doesn't need to be = x0

for t = 1:(length(T)-1)
    r = 2; % reference roll angle (rad)
    if T(t) > 15
        r = 0.5;
    end
    e = x(:,t) - x_ref(:,t);
    u = kx'*x(:,t) + kr'*r; % requires full-state feedback

    kx_dot  = -gamma_x*x(:,t)*e'*P*signB; % adaptive controller
    kr_dot  = -gamma_r*r*e'*P*signB; % adaptive controller
    kx = kx + dt * kx_dot;
    kr = kr + dt * kr_dot;

    x_ref_dot = A_ref*x_ref(:,t) + B_ref*r;
    x_ref(:,t+1) = x_ref(:,t) + x_ref_dot * dt;

    x_dot = A*x(:,t) + B*u;
    x(:,t+1) = x(:,t) + x_dot * dt;
end

plot(T,x(1,:),'b',T,x_ref(1,:),'r')
hold on;
plot(T,x(2,:),'--b',T,x_ref(2,:),'--r')
legend('actual physical system roll angle', 'reference model (virtual system) roll angle', ...
    'actual physical system roll rate', 'reference model (virtual system) roll rate')