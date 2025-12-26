clc; clear; close all
m = 1;
k = 1;
c = 1;
A = [0 1; -k/m -c/m];
B = [0; 1/m];
C = [0 1];
D = 0;

sys = ss(A,B,C,D);
s = tf('s');
% G = 1/(m*s^2+c*s+k);
G = s/(m*s^2+c*s+k);
K = 1;
% sys = minreal(K*G/(1+K*G));

subplot(3,1,1)
rlocus(G)

subplot(3,1,2)
rlocus(sys)
% pzplot(sys)

t0 = 0;
dt = 0.01;
Tf = 50;
x = zeros(2,length(t0:dt:Tf));
x0 = [-1;1];
x(:,1) = x0;
T = t0:dt:Tf;
for t = 1:(length(T)-1)
    u = (5-x(2,t));
    x_dot = A*x(:,t) + B*u;
    x(:,t+1) = x(:,t) + x_dot*dt;
end

subplot(3,1,3)
plot(T,x(1,:),'-r',T,x(2,:),'-b')
legend('pos (m)', 'vel (m/s)')
xlabel('time (s)')
ylabel('state')
grid on
grid minor