Longitudinal_Stability_Analysis
close all

%% Short-Period
Bde_sp = B(2:3,1); % de for alpha and q
C_sp = [0 1]; % q
figure(1)
rlocus(A_sp,Bde_sp,C_sp,0);
[num,den] = ss2tf(A_sp,Bde_sp,C_sp,0);
q_de_tf = tf(num,den);

de_u_tf = tf(-1,[0.1 1]); % actuator dynamics

G_q_u = de_u_tf*q_de_tf;
figure(2)
rlocus(G_q_u)