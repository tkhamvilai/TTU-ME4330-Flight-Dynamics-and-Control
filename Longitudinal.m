Aircraft_Parameters
rho = 1.225;
Q = 0.5*rho*u0^2;
S = params.S;
m = params.m;
c = params.c;
M = 0.158; % mach number
Jy = params.J(2,2);
g = params.g;

Z_w = -Q*S/m/u0*(params.CD0 + params.CDa);
Z_q = -Q*S*c/2/m/u0*params.CLq;
Z_u = -Q*S/m/u0*(2*params.CL0 + params.CLM*M);
M_w = Q*S*c/Jy/u0*params.Cma;
M_w_dot = Q*S*c/Jy/u0*c/2/u0*params.Cma_dot;
M_q = Q*S*c/Jy*c/2/u0*params.Cmq;
M_u = Q*S*c/Jy/u0*params.CmM*M;
X_w = Q*S/m/u0*(params.CL0 - params.CD0);
X_u = -Q*S/m/u0*(2*params.CD0 + params.CDM*M);

% x = [u w q theta]
A = [X_u  X_w 0 -g;
     Z_u  Z_w u0 0;
     M_u+M_w_dot*Z_u M_w+M_w_dot*Z_w M_q+M_w_dot*u0 0;
     0 0 1 0];
B = [0 params.P_max/u0;
     params.CLde 0;
     params.Cmde 0;
     0 0];