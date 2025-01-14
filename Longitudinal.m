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

% x = [u w q theta]
A = [X_u  X_w 0 -g;
     Z_u  Z_w u0 0;
     M_u+M_w_dot*Z_u M_w+M_w_dot*Z_w M_q+M_w_dot*u0 0;
     0 0 1 0];
B = [0 Cx_dpt;
     CL_de Cz_dpt;
     Cm_de Cm_dpt;
     0 0];