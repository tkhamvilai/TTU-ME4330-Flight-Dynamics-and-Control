% Linear Control
% What if I don't know or don't want to know what rlocus is
% and I don't want to tune PID gains either
% Is there a way for the code, the algorithm, or the control theory,
% to tell us what gain values should be?
% Yes, u = -Kx
% where K = -inv(R)*B'*P and
% P is solved from 0 = P*A + A'*P + Q - P*B*inv(R)*B'*P
% Q and R are trade-off matrices between state errors and control effort.
% We pick Q and R ourselves 
% an identity matrices work most of the time for Q and R.

% To implement this, MATLAB has a function called
% lqr() (Linear Quadratic Regulator) to directly give us K

clear; clc; close all
m = 1; % mass
k = -1; % spring stiffness
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

t0 = 0;
dt = 0.01;
Tf = 20;
T = t0:dt:Tf;
x = zeros(2,length(T));
x0 = [1;-1]; % initial state
x(:,1) = x0;

for t = 1:(length(T)-1)
    r_pos = 3; % for the linear control to track a reference, we need another gain
    
    
    [K,~,~] = lqr(A,B,eye(2),1); % linear control with Q and R being identity
    Kr = -inv(C1*inv(A-B*K)*B); % gain for the reference signal
    % u = -K*x(:,t); % we a full-state feedback here, i.e., both position and velocity
    % u = r_pos - K*x(:,t); % to track a reference
    u = Kr*r_pos - K*x(:,t); % to track a reference

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