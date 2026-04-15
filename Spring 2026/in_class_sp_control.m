clear; clc; close all

%% simulation time
t0 = 0; % initial time (s)
Tf = 10; % final time (s)
dt = 0.001; % timestep (s)
T = t0:dt:Tf; % time vector

%% short-period mode system dynamics
A_sp = [-1.9546 1; -7.7519 0];
B_sp = [-0.1395; -24.7036];
% C = [1 0]; % we will at the first state (angle of attack) as an output of the transfer function
C = [1 0;
     0 1]; % both angle of attack and pitch rate can be measured.
D = 0;

% convert state-space model to transfer function model
tf(ss(A_sp, -B_sp, C, D)) % -B_sp because of the opposite sign between u and delta_e

% Include the actuator dynamics using 1-st order model
tau = 0.05; % the acutator will take 0.05 seconds to get to the ~60% of the command value, 
% e.g., if you tell the elevator to deflect by 1 deg, it will take 0.05 seconds to deflect by 0.6 deg.
A = [-1.9546 1 -0.1395; 
     -7.7519 0 -24.7036;
         0   0 -1/tau];
B = [0; 0; -1];
C = [1 0 0;
     0 1 0];

tf(ss(A, B, C, D))

% initialize state vector [angle of attack (rad); pitch rate (rad/s)]
x = zeros(3,length(T)); % time-series of state vector
x(:,1) = [0; 1; 0]; % initial state

% de = zeros(1,length(T)); % time-series of elevator deflection
u = zeros(1,length(T)); % time-series of control vector

err_I = 0; % initial value of the integral error

%% simulation run
for i = 1:length(T)
    % get feedback date (angle of attack and pitch rate)
    alpha = x(1,i);
    q = x(2,i);

    % u(i) = 0; % doing nothing
    u(i) = -5*q; % pitch rate stability augmentation system
    
    % alpha_desired = 3*pi/180; % desired angle of attack (rad)
    % Kp = 1;
    % u(i) = Kp*(alpha_desired - alpha); % P-control signal
    % Ki = 1;
    % err = alpha_desired - alpha;
    % err_I = err_I + err * dt;
    % u(i) = Kp * err + Ki * err_I;
    
    % de(i) = -u(i); % positive surface deflection leads to a negative rotation of an aircraft
    % de(i) = clip(de(i), -15*pi/180, 15*pi/180); % to simulate the physical limit of the elevator to +- 15 deg.
    % x_dot = A_sp * x(:,i) + B_sp * de(i);
    x_dot = A * x(:,i) + B * u(i);
    x(:,i+1) = x(:,i) + x_dot * dt; % update state vector using Euler integration
end

%% plot
subplot(2,2,1)
plot(T, x(1,1:length(T))*180/pi, '-r'); hold on
% plot(T, alpha_desired*180/pi * ones(1,length(T)), '--r')
xlabel('Time (s)'); 
ylabel('Angle of Attack, \Delta\alpha (deg)')
grid on; grid minor

subplot(2,2,2)
plot(T, x(2,1:length(T))*180/pi, '-r')
xlabel('Time (s)'); 
ylabel('Pitch rate, \Delta q (deg/s)')
grid on; grid minor

subplot(2,2,3)
plot(T, u(1,1:length(T))*180/pi, '-r')
xlabel('Time (s)'); 
ylabel('Control signal (deg)')
grid on; grid minor

subplot(2,2,4)
% plot(T, de(1,1:length(T))*180/pi, '-r')
plot(T, x(3,1:length(T))*180/pi, '-r'); hold on
xlabel('Time (s)'); 
ylabel('Elevator Deflection, \Delta_e (deg)')
grid on; grid minor