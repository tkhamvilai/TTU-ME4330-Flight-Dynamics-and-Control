Longitudinal_Stability_Analysis
close all

%% Alt with Elevator
% x = [u; alpha; q; theta; h]

% AA = [-0.0082354 18.938   0 -32.17  0;
%      -0.00025617 -0.56761 1  0     0;
%        1.3114e-5 -1.4847  -0.47599  0 0;
%                0     0    1  0     0;
%                0 -500     0 500     0];
% BB = [0; 0; -0.019781; 0; 0];
% C = [0 0 0 180/pi 0; 
%      0 0 180/pi 0 0];

AA = [A zeros(4,1); 0 -u0 0 u0 0];
BB = [B(:,1); 0];
C = [0 0 0 1 0; 
     0 0 1 0 0];
plant = ss(AA,BB,C,0);

aa = -10; ba = 10; ca = -1; % actuator dynamics
actua = ss(aa,ba,ca,0);
sys1 = series(actua,plant);

[a,b,c,d] = ssdata(sys1);
acl = a - b*[1 0.2]*c;
ch = [0 0 0 0 1 0];
sys2 = ss(acl,1*b,ch,0);

G = zpk([-0.3 -0.5],[-1],1);
sys3 = series(G,sys2);
[a,b,c,d] = ssdata(sys3);
acl = a - 0.6*b*c;
closed = ss(acl,0.6*b,c,0);

figure(); rlocus(sys3)
figure(); step(closed,30)