clear; clc; close all

%% time-related parameters
% integrate numerically x_dot = t^2
t0 = 0; % initial time
tf = 10; % final time
dt = 0.01; % time step
tspan = t0:dt:tf; % time span

%% pre-allocation
x = zeros(length(tspan),1); % a column vector of the size of tspan x 1

%% simulation
itr = 1;
for t = tspan
    x_dot = t^2;
    if t < tspan(end) % do an integration if we are not at the final time
        x(itr+1,1) = x(itr,1) + x_dot * dt;
    end
    itr = itr + 1; % increment the iterator
end

%% analytical solution
x0 = 0;
x_analy = x0 + tspan.^3 / 3; % integrate this by hand

%% visualization
plot(tspan',x,'-r',tspan',x_analy,'--b')
grid on; grid minor
xlabel('time (s)')
ylabel('x(t)')
title('x(t) vs time')
legend('x(t) numerical','x(t) analytical')