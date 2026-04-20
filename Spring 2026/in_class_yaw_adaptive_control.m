clear; clc; close all
% simulate and control the unstable spiral mode

%% simulation time
t0 = 0; % initial time (s)
Tf = 75; % final time (s)
dt = 0.001; % timestep (s)
T = t0:dt:Tf; % time vector

%% sprial mode
% actual system
A = 0.25;
B = 0.5;

% reference model
A_ref = -1;
B_ref = 1;

%% initialization
x = zeros(1,length(T)); % time-series of state vector
x(:,1) = 1; % initial yaw rate in rad/s
u = zeros(1,length(T)); % time-series of control vector
dr = zeros(1,length(T)); % time-series of rudder deflection
k = zeros(1,length(T)); % initial gain value is zero

k_r = zeros(1,length(T)); % initial gain value is zero
k_cmd = zeros(1,length(T)); % initial gain value is zero

x_ref = zeros(1,length(T)); % time-series of reference model's state vector

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
    t = i * dt;
    cmd = 0.5 * sin(t); % command signal
    % u(i) = -k(i)*x(:,i);
    u(i) = -k_r(i)*x(:,i) - k_cmd(i)*cmd;
    dr(i) = -u(i);

    % reference model
    x_ref_dot = A_ref * x_ref(:,i) + B_ref * cmd;
    x_ref(:,i+1) = x_ref(:,i) + x_ref_dot * dt;
    
    % system dynamics (we don't know anything about it)
    x_dot = A * x(:,i) + B * dr(i);
    x(:,i+1) = x(:,i) + x_dot * dt; % update state vector using Euler integration

    % gain dynamic
    % k_dot = -x(:,i)*x(:,i); % k_dot = -r^2
    % k(:,i+1) = k(i) + k_dot * dt;

    k_r_dot = x(:,i)*(x_ref(:,i) - x(:,i));
    k_r(:,i+1) = k_r(:,i) + k_r_dot * dt;

    k_cmd_dot = cmd*(x_ref(:,i) - x(:,i));
    k_cmd(:,i+1) = k_cmd(:,i) + k_cmd_dot * dt;
end

%% plot
subplot(2,2,1)
plot(T, x(1,1:length(T)), '-r'); hold on
plot(T, x_ref(1,1:length(T)), '-b');
xlabel('Time (s)'); 
ylabel('Yaw rate, \Delta r (rad/s)')
legend('actual system', 'reference system')
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
% plot(T, k(1,1:length(T))*180/pi, '-r'); hold on
plot(T, k_r(1,1:length(T)), '-r'); hold on
plot(T, k_cmd(1,1:length(T)), '-b'); 
xlabel('Time (s)'); 
ylabel('Gain value, k')
legend('K r','K cmd')
grid on; grid minor