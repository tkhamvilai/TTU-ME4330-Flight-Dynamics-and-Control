clear; clc; close all
% simulate and control the unstable spiral mode

%% simulation time
t0 = 0; % initial time (s)
Tf = 10; % final time (s)
dt = 0.001; % timestep (s)
T = t0:dt:Tf; % time vector

%% sprial mode
A = 0.25;
B = 0.5;

%% initialization
x = zeros(1,length(T)); % time-series of state vector
x(:,1) = 1; % initial yaw rate in rad/s
u = zeros(1,length(T)); % time-series of control vector
dr = zeros(1,length(T)); % time-series of rudder deflection
k = zeros(1,length(T)); % initial gain value is zero

%% simulation run
for i = 1:length(T)
    % k = 1; % some randome gain value
    % k = -10; % we need k < -2 for stability.
    % k < -2 is derived from the assumption that we know
    % the value of all aerodynamic coefficients

    % If we don't know the value of all aerodynamic coefficients
    % we will keep changing k in real time
    % k will be changing according to k_dot = -r^2
    
    % controller
    u(i) = -k(i)*x(:,i);
    dr(i) = -u(i);
    
    % system dynamics
    x_dot = A * x(:,i) + B * dr(i);
    x(:,i+1) = x(:,i) + x_dot * dt; % update state vector using Euler integration

    % gain dynamic
    k_dot = -x(:,i)*x(:,i); % k_dot = -r^2
    k(:,i+1) = k(i) + k_dot * dt;
end

%% plot
subplot(2,2,1)
plot(T, x(1,1:length(T))*180/pi, '-r'); hold on
xlabel('Time (s)'); 
ylabel('Yaw rate, \Delta r (deg/s)')
grid on; grid minor

subplot(2,2,2)
plot(T, u(1,1:length(T))*180/pi, '-r'); hold on
xlabel('Time (s)'); 
ylabel('Control signal, u (deg)')
grid on; grid minor

subplot(2,2,3)
plot(T, dr(1,1:length(T))*180/pi, '-r'); hold on
xlabel('Time (s)'); 
ylabel('Rudder Deflection, \delta_r (deg)')
grid on; grid minor

subplot(2,2,4)
plot(T, k(1,1:length(T))*180/pi, '-r'); hold on
xlabel('Time (s)'); 
ylabel('Gain value, k')
grid on; grid minor