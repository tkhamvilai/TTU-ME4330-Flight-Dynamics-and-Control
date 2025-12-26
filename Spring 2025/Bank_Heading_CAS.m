Lateral_Stability_Analysis
close all

%% Bank + Heading
AA = [A zeros(4,1); 0 0 1 0 0];
BB = [B; 0 0];
Q = eye(5);
R = eye(2);
[K,S,P] = lqr(AA,BB,Q,R)
A_cl = AA - BB*K;
sys = ss(A_cl,[],eye(5),0);
lsim(sys,[],0:0.01:10,[0,0,0,deg2rad(30),deg2rad(45)])