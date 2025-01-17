clear; clc; close all
AircraftParameters
AircraftInitialization

%% Longitudinal
rho = 1.225;
u0 = vel(1);
Q = 0.5*rho*u0^2;
Mach = 0.158;
Jy = J(2,2);

Z_w = -Q*S/m/u0*(CD_0 + CD_alpha);
Z_q = -Q*S*c/2/m/u0*CL_q;
Z_u = -Q*S/m/u0*(2*CL_0 + CL_Mach*Mach);
M_w = Q*S*c/Jy/u0*Cm_alpha;
M_w_dot = Q*S*c/Jy/u0*c/2/u0*Cm_alpha_dot;
M_q = Q*S*c/Jy*c/2/u0*Cm_q;
M_u = Q*S*c/Jy/u0*Cm_Mach*Mach;
X_w = Q*S/m/u0*(CL_0 - CD_0);
X_u = -Q*S/m/u0*(2*CD_0 + CD_Mach*Mach);
X_alpha = X_w*u0;
Z_alpha = Z_w*u0;
M_alpha = M_w*u0;
M_alpha_dot = M_w_dot*u0;
Z_de = -Q*S/m*CL_de;
M_de = Q*S*c/Jy*Cm_de;

% x = [u; w; q; theta]
% A = [X_u X_w 0 -g;
%      Z_u Z_w u0 0;
%      M_u+M_w_dot*Z_u M_w+M_w_dot*Z_w M_q+M_w_dot*u0 0;
%      0 0 1 0];

% x = [u; alpha; q; theta]
A = [X_u X_alpha 0 -g;
     Z_u/u0 Z_alpha/u0 1 0;
     M_u+M_alpha_dot*Z_u/u0 M_alpha+M_alpha_dot*Z_alpha/u0 M_q+M_alpha_dot 0;
     0 0 1 0];
B = [0 Cx_dpt;
     Z_de/u0 Cz_dpt;
     M_de + M_alpha_dot*Z_de/u0 Cm_dpt;
     0 0];
sys = ss(A,[],eye(4),0);
damp(sys)

tspan = 0:0.01:250;
IC = [u0;0;deg2rad(1);deg2rad(0)];
figure(1)
lsim(sys,[],tspan,IC);

%% Phugoid / Long-Period Mode
% x = [u; theta]
A_pg = [X_u -g; -Z_u/u0 0];
sys_pg = ss(A_pg,[],eye(2),0);
damp(sys_pg)
IC_pg = [u0;deg2rad(0)];
figure(2)
lsim(sys_pg,[],tspan,IC_pg);
omega_pg = sqrt(-Z_u*g/u0);
damp_pg = -X_u/2/omega_pg;

%% Short-Period Mode
% x = [alpha; q]
A_sp = [Z_alpha/u0 1; M_alpha + M_alpha_dot*Z_alpha/u0 M_q + M_alpha_dot];
sys_sp = ss(A_sp,[],eye(2),0);
damp(sys_sp)
IC_sp = [0;deg2rad(1)];
figure(3)
lsim(sys_sp,[],tspan,IC_sp);
omega_sp = sqrt(-Z_alpha*M_q/u0 - M_alpha);
damp_sp = -(M_q + M_alpha_dot + Z_alpha/u0)/(2*omega_sp);

% T_half = 0.69/abs(real)
% Period = 2*pi/abs(img)
% N_half = T_half/P = 0.11*abs(img)/abs(real)