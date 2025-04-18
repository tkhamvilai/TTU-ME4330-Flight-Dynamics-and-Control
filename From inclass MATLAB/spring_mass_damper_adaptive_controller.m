% Adaptive Control
% What if we want to use the linear controller
% but we don't know anything about the system
% i.e., we don't know the value of the A and B matrices
% We know that our system is in the form of
% x_dot = Ax + Bu but we don't know the value of A and B
% Spoiler: we keep changing the gains during the execution
% K will not be a constant number anymore
% Note: PID is also a model-free but we need to tune kp, ki, kd terms

%% System parameters
% only use in the simulation but not in the control design
Lp = -0.5; % roll damping
Lda = 2; % roll moment coefficient due to aileron deflections

%% Roll rate
A = Lp;
B = Lda;

%% Adaptive control
% below are design parameters (our choice)
gamma_x = 1;
gamma_r = 1;
% Q = 1;
A_ref = -4;
B_ref = 4;

% this is required to be known from the system
% we know that +aileron deflection gives +roll moment
% so sign B is positive
signB = 1;

% initial gains (also our choice)
kx = 0;
kr = 0;

% P matrix is a solve to A_ref'P + PA_ref + Q = 0 (Lyapunov equantion)
% P = lyap(A_ref',Q);
% however, this problem is a scalar so P = -Q/2A_ref (if we pick Q = -2A_ref -> P = 1)
P = 1;

%% Simulation
t0 = 0;
dt = 0.01;
Tf = 20;
T = t0:dt:Tf;
x = zeros(1,length(T));
x0 = 0.5; % initial roll rate
x(:,1) = x0;

x_ref = zeros(1,length(T)); % reference model, note that init x_ref(:,1) doesn't need to be = x0

for t = 1:(length(T)-1)
    r = 2; % reference roll rate
    if T(t) > 15
        r = 1.5;
    end
    e = x(:,t) - x_ref(:,t);
    u = kx*x(:,t) + kr*r; % requires full-state feedback

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
legend('actual physical system', 'reference model (virtual system)')