clear; clc; close all
Yv = -0.254;
Yb = -45.72;
Yp = 0;
Yr = 0;
Nv = 0.025;
Nb = 4.49;
Np = -0.35;
Nr = -0.76;
Lv = -0.091;
Lb = -16.02;
Lp = -8.4;
Lr = 2.19;
u0 = 176; %ft/s
g = 32.2; % ft/st^2
theta_0 = 0; % pitch angle at cruise

% state = beta, p, r, phi
A_lateral = [Yb/u0 Yp/u0 -(1-Yr/u0) g/u0*cos(theta_0);
            Lb Lp Lr 0;
            Nb Np Nr 0;
            0 1 0 0];
t0 = 0;
dt = 0.01;
Tf = 10;
ind = 1;

state_lateral = zeros(4,length(t0:dt:Tf));
% initial difference between the states and the equilibrium
state_init = [pi/12; 0; 0; deg2rad(5)]; % state = beta, p, r, phi
state_lateral(:,1) = state_init;

for i = t0:dt:Tf
    state_lateral_dot = A_lateral*state_lateral(:,ind);
    state_lateral(:,ind+1) = state_lateral(:,ind) + state_lateral_dot*dt;
    ind = ind + 1;
end
subplot(4,1,1)
plot(t0:dt:Tf, state_lateral(1,1:end-1), '-r'); hold on;
ylabel('delta beta')
xlabel('time')
subplot(4,1,2)
plot(t0:dt:Tf, state_lateral(2,1:end-1), '-b'); hold on;
ylabel('delta p')
xlabel('time')
subplot(4,1,3)
plot(t0:dt:Tf, state_lateral(3,1:end-1), '-g'); hold on;
ylabel('delta r')
xlabel('time')
subplot(4,1,4)
plot(t0:dt:Tf, state_lateral(4,1:end-1), '-k'); hold on;
ylabel('delta phi')
xlabel('time')

damp(A_lateral)

% Spiral
A_spiral = (Lb*Nr - Lr*Nb)/Lb;
damp(A_spiral)

% Roll
A_roll = Lp;
damp(A_roll)

% Dutch Roll
A_dutch_roll = [Yb/u0 Yr/u0-1;
                Nb    Nr];
damp(A_dutch_roll)