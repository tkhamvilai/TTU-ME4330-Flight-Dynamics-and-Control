Longitudinal_Stability_Analysis
close all

%% Alt with Elevator
AA = [A zeros(4,1); 0 -u0 0 u0 0];
BB = [B(:,1); 0];
Q = eye(5);
R = eye(1);
[K,S,P] = lqr(AA,BB,Q,R)