clear; clc; close all

% integrate numerically x_dot = t^2
t0 = 0; % initial time
tf = 10; % final time
dt = 0.01; % time step
tspan = t0:dt:tf; % time span

x = zeros(length(tspan),1); % a column vector of the size of tspan x 1

itr = 1;
for t = tspan
    x_dot = t^2;
    if t < tspan(end)
        x(itr+1,1) = x(itr,1) + x_dot * dt;
    end
    itr = itr + 1; % increment the iterator
end

plot(tspan',x)