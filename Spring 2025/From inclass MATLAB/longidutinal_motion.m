clear; clc; close all
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

A_longidutal = [Xu Xw 0 -g; % u
                Zu Zw u0 0; % w
                Mu+Mwdot*Zu Mw+Mwdot*Zw Mq+Mwdot*u0 0; % q
                0 0 1 0]; % theta
damp(A_longidutal)

A_long_period = [Xu -g; -Zu/u0 0];
A_short_period = [Za/u0 1; Ma + Madot/u0*Za Mq+Madot];
damp(A_long_period)
damp(A_short_period)

t0 = 0;
Tf = 100;
dt = 0.1;
state_longitudinal = zeros(4,length(t0:dt:Tf));
% initial difference between the states and the equilibrium
state_init = [0; 0; 0.5; pi/6];
state_longitudinal(:,1) = state_init;

state_long_period = zeros(2,length(t0:dt:Tf)); % delta u and delta theta
state_short_period = zeros(2,length(t0:dt:Tf));% delta w and delta q

state_long_period(:,1) = [state_init(1); state_init(4)];
state_short_period(:,1) = [state_init(2); state_init(3)];
ind = 1;
for i = t0:dt:Tf
    % x_(t+1) = x_t + x_dot*dt
    % x_dot = Ax
    % full linearized model
    state_longitudinal(:,ind+1) = state_longitudinal(:,ind) + dt*(A_longidutal*state_longitudinal(:,ind));
    
    % Long-period mode approximation
    state_long_period(:,ind+1) = state_long_period(:,ind) + dt*(A_long_period*state_long_period(:,ind));

    % Short-period mode approximation
    state_short_period(:,ind+1) = state_short_period(:,ind) + dt*(A_short_period*state_short_period(:,ind));
    ind = ind+1;
end
subplot(4,1,1)
plot(t0:dt:Tf, state_longitudinal(1,1:end-1), '-r'); hold on;
plot(t0:dt:Tf, state_long_period(1,1:end-1), '--r');
ylabel('delta u')
xlabel('time')
subplot(4,1,2)
plot(t0:dt:Tf, state_longitudinal(2,1:end-1), '-b'); hold on;
plot(t0:dt:Tf, state_short_period(1,1:end-1), '--b');
ylabel('delta w')
xlabel('time')
subplot(4,1,3)
plot(t0:dt:Tf, state_longitudinal(3,1:end-1), '-g'); hold on;
plot(t0:dt:Tf, state_short_period(2,1:end-1), '--g');
ylabel('delta q')
xlabel('time')
subplot(4,1,4)
plot(t0:dt:Tf, state_longitudinal(4,1:end-1), '-k'); hold on;
plot(t0:dt:Tf, state_long_period(2,1:end-1), '--k');
ylabel('delta theta')
xlabel('time')