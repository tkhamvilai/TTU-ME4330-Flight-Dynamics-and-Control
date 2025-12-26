clear; clc; close all
Lp = -0.5;
Lda = 2;

Lp_ref = -4;
Lda_ref = 4;

t0 = 0;
dt = 0.01;
Tf = 20;
T = t0:dt:Tf;
p = zeros(1,length(T));
p0 = 1; % initial state
p(:,1) = p0;

p_ref = zeros(1,length(T));

gamma_p = 1;
gamma_r = 1;
kp = 0;
kr = 0;

for t = 1:(length(T)-1)
    r = 2;
    e = p(t) - p_ref(t);   
    da = kp*p(t) + kr*r;

    p_dot = Lp*p(t) + Lda*da;
    p_ref_dot = Lp_ref*p_ref(t) + Lda_ref*r;

    kp_dot = -gamma_p*p(t)*e;
    kr_dot = -gamma_p*r*e;

    kp = kp + dt*kp_dot;
    kr = kr + dt*kr_dot;

    p(t+1) = p(t) + dt*p_dot;
    p_ref(t+1) = p_ref(t) + dt*p_ref_dot;
end

plot(T,p,'-b',T,p_ref,'r')