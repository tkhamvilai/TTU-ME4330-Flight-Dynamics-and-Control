Lateral_Stability_Analysis
close all

%% Lateral
C = [0 1 0 0; % p
     0 0 1 0]; % r
[num,den] = ss2tf(A,B(:,1),C(1,:),0);
p_da_tf = tf(num,den);
[num,den] = ss2tf(A,B(:,2),C(1,:),0);
p_dr_tf = tf(num,den);
[num,den] = ss2tf(A,B(:,1),C(2,:),0);
r_da_tf = tf(num,den);
[num,den] = ss2tf(A,B(:,2),C(2,:),0);
r_dr_tf = tf(num,den);

% figure(1)
% subplot(4,1,1)
% rlocus(-p_da_tf); % +da = -p
% subplot(4,1,2)
% rlocus(p_dr_tf); % +dr = +p
% subplot(4,1,3)
% rlocus(-r_da_tf); % +da = -r
% subplot(4,1,4)
% rlocus(-r_dr_tf); % +dr = -r

d_u_tf = tf(-1,[0.1 1]); % actuator dynamics (both aileron and rudder)

p_ua_tf = p_da_tf*d_u_tf;
p_ur_tf = p_dr_tf*d_u_tf;
r_ua_tf = r_da_tf*d_u_tf;
r_ur_tf = r_dr_tf*d_u_tf;

% figure(2)
% subplot(4,1,1)
% rlocus(p_ua_tf);
% subplot(4,1,2)
% rlocus(p_ur_tf);
% subplot(4,1,3)
% rlocus(r_ua_tf);
% subplot(4,1,4)
% rlocus(r_ur_tf);

% same actuator dynamics but in ss form
Aa = [-10 0; 0 -10];
Ba = [10 0; 0 10];
Ca = [-1 0; 0 -1];

plant = ss(A,B,C,0);
actua = ss(Aa,Ba,Ca,0);
sys1 = series(actua,plant);

% washout filter
Aw = -1;
Bw = [0 1];
Cw = [0;-1];
Dw = [1 0; 0 1]; % p, washed-r
wash = ss(Aw,Bw,Cw,Dw);
sys2 = series(sys1,wash);

[num,den] = ss2tf(sys2.A,sys2.B(:,1),sys2.C(1,:),0);
p_ua_tf_wo = zpk(tf(num,den));
[num,den] = ss2tf(sys2.A,sys2.B(:,2),sys2.C(1,:),0);
p_ur_tf_wo = zpk(tf(num,den));
[num,den] = ss2tf(sys2.A,sys2.B(:,1),sys2.C(2,:),0);
r_ua_tf_wo = zpk(tf(num,den));
[num,den] = ss2tf(sys2.A,sys2.B(:,2),sys2.C(2,:),0);
r_ur_tf_wo = zpk(tf(num,den));

figure(3)
subplot(4,1,1)
rlocus(p_ua_tf_wo)
subplot(4,1,2)
rlocus(p_ur_tf_wo)
subplot(4,1,3)
rlocus(r_ua_tf_wo)
subplot(4,1,4)
rlocus(r_ur_tf_wo)

% close roll-rate loop
Kp = 1; % roll-rate gain
A_cl = sys2.A - sys2.B(:,1)*Kp*sys2.C(1,:); % feedback with p
sys3 = ss(A_cl,sys2.B(:,2),sys2.C,0);

[num,den] = ss2tf(A_cl,sys2.B(:,2),sys2.C(1,:),0);
p_ur_tf_cl = zpk(tf(num,den));
[num,den] = ss2tf(A_cl,sys2.B(:,1),sys2.C(2,:),0);
r_ua_tf_cl = zpk(tf(num,den));
[num,den] = ss2tf(A_cl,sys2.B(:,2),sys2.C(2,:),0);
r_ur_tf_cl = zpk(tf(num,den));

figure(4)
subplot(3,1,1)
rlocus(p_ur_tf_cl)
subplot(3,1,2)
rlocus(r_ua_tf_cl)
subplot(3,1,3)
rlocus(r_ur_tf_cl)

% close yaw-rate loop
Kr = 1; % yaw-rate gain
A_cl2 = A_cl - sys3.B*Kr*sys3.C(2,:);
sys4 = ss(A_cl2,[],sys2.C,0);
damp(sys4)