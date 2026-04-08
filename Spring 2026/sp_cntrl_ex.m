clear; clc; close all
%% Simulation time
t0 = 0;
Tf = 10;
dt = 0.01;
T = t0:dt:Tf;

%% short-period dynamics
A_sp = [ -1.9546    1.0000;
         -7.7519         0];
B_sp = [-0.1395;
        -24.7036];

A = [-1.9546    1.0000 -0.1395;
     -7.7519         0 -24.7036;
           0         0 -10];

% x = zeros(2,length(T)); % alpha; q
% x(:,1) = [0; 2];

x = zeros(3,length(T)); % alpha; q
x(:,1) = [0; 2; 0];

%% Simulation run
for i = 1:length(T)
    alpha = x(1,i);
    q = x(2,i);
    de = q;
    u = q;
    % x_dot = A_sp * x(:,i) + B_sp*de;
    x_dot = A * x(:,i) + [0;0;1]*u;
    x(:,i+1) = x(:,i) + x_dot * dt;
end

%% plot
plot(T, x(1,1:length(T)), '-r'); hold on
plot(T, x(2,1:length(T)), '-b');
legend('angle of attack \alpha','pitch rate q');
xlabel('time (s)');
ylabel('states');
grid on; grid minor