clear; clc; close all
% A = [-0.0082354 18.938 -32.17 0 5.9022e-5;
%      -0.00025617 -0.56761 0 1 2.2633e-6;
%      0 0 0 1 0;
%      1.3114e-5 -1.4847 0 -0.47599 -1.4947e-7;
%      0 -500 500 0 0];
A = [-0.0082354 18.938 -32.17 0 0;
     -0.00025617 -0.56761 0 1 0;
     0 0 0 1 0;
     1.3114e-5 -1.4847 0 -0.47599 0;
     0 -500 500 0 0];
B = [0; 0; 0; -0.019781; 0];
C = [0 0 180/pi 0 0; 0 0 0 180/pi 0];
plant = ss(A,B,C,0);

aa = -10; ba = 10; ca = -1; % actuator dynamics
actua = ss(aa,ba,ca,0);
sys1 = series(actua,plant);

[a,b,c,d] = ssdata(sys1);
acl = a - b*[3 2.5]*c;
ch = [0 0 0 0 1 0];
sys2 = ss(acl,3*b,ch,0);

lead = ss(-2.4,2.4,-0.875,1);
sys3 = series(lead,sys2);
[a,b,c,d] = ssdata(sys3);

lag = ss(-0.01,0.01,4,1);
sys4 = series(lag,sys3);
[a,b,c,d] = ssdata(sys4);
acl = a - 0.3333*b*c;
closed = ss(acl,0.3333*b,c,0);
step(closed,30)