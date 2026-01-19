clear; clc; close all
flightSim_createAircraft

%% Lateral-Related Aircraft Parameters
% general
g    = Environment.Gravity;
rho  = Environment.Density;
u0   = state.U;
Q    = 0.5*rho*u0^2;
Mach = u0/Environment.SpeedOfSound;
Jx   = state.Inertia(1,1).Variables;
Jz   = state.Inertia(3,3).Variables;
Jxz  = abs(state.Inertia(1,3).Variables);
m    = state.Mass;
S    = aircraft.ReferenceArea;
c    = aircraft.ReferenceLength;
b    = aircraft.ReferenceSpan;
alpha_0 = state.Alpha;
beta_0  = state.Beta;

% stability and control coefficients
CY_0      = getCoefficient(aircraft,"CY","Zero");
CY_beta   = getCoefficient(aircraft,"CY","Beta");
CY_p      = getCoefficient(aircraft,"CY","pb2V");
CY_r      = getCoefficient(aircraft,"CY","rb2V");

Cl_0      = getCoefficient(aircraft,"Cl","Zero");
Cl_beta   = getCoefficient(aircraft,"Cl","Beta");
Cl_p      = getCoefficient(aircraft,"Cl","pb2V");
Cl_r      = getCoefficient(aircraft,"Cl","rb2V");

Cn_0      = getCoefficient(aircraft,"Cn","Zero");
Cn_beta   = getCoefficient(aircraft,"Cn","Beta");
Cn_beta3  = getCoefficient(aircraft,"Cn","Beta3");
Cn_p      = getCoefficient(aircraft,"Cn","pb2V");
Cn_q      = getCoefficient(aircraft,"Cn","qcV");
Cn_r      = getCoefficient(aircraft,"Cn","rb2V");

% stability and control derivatives
Y_beta  = Q*S/m*CY_beta;
Y_p     = Q*S/m/u0*CY_p*(b/2);
Y_r     = Q*S/m/u0*CY_r*(b/2);

L_beta  = Q*S*b/Jx*Cl_beta;
L_p     = Q*S*b/Jx/u0*Cl_p*(b/2);
L_r     = Q*S*b/Jx/u0*Cl_r*(b/2);

N_beta  = Q*S*b/Jz*Cn_beta;
N_beta3 = Q*S*b/Jz*Cn_beta*(3*beta_0^2);
N_p     = Q*S*b/Jz/u0*Cn_p*(b/2);
N_q     = Q*S*b/Jz/u0*Cn_p*(c);
N_r     = Q*S*b/Jz/u0*Cn_r*(b/2);

% rol-yaw coupling
Gamma        = 1 - Jxz^2/Jx/Jz;
L_beta_prime = L_beta/Gamma + Jxz/Jx*N_beta;
L_p_prime    = L_p/Gamma    + Jxz/Jx*N_p;
L_r_prime    = L_r/Gamma    + Jxz/Jx*N_r;
N_beta_prime = N_beta/Gamma + Jxz/Jz*L_beta;
N_p_prime    = N_p/Gamma    + Jxz/Jz*L_p;
N_r_prime    = N_r/Gamma    + Jxz/Jz*L_r;

%% Lateral Dynamics
% x = [beta; p; r; phi]
A = [Y_beta/u0 Y_p/u0 -(1-Y_r/u0) g/u0;
     L_beta_prime L_p_prime L_r_prime 0;
     N_beta_prime N_p_prime N_r_prime 0;
     0 1 0 0];
% B = [Y_da/u0 Y_dr/u0;
%      L_da L_dr;
%      N_da N_dr;
%      0 0];

sys = ss(A,[],eye(4),0);
damp(sys)

tspan = 0:0.01:50;
IC = [deg2rad(1);deg2rad(1);deg2rad(1);deg2rad(0)];
figure(1)
lsim(sys,[],tspan,IC);

%% Spiral
% x = [r]
A_spi = N_r_prime - L_r_prime*N_beta_prime/L_beta_prime;
sys_spi = ss(A_spi,[],eye(1),0);
damp(sys_spi)
IC_spi = [deg2rad(1)];
figure(2)
lsim(sys_spi,[],tspan,IC_spi);

%% Roll
% x = [p]
A_roll = L_p_prime;
sys_roll = ss(A_roll,[],eye(1),0);
damp(sys_roll)
IC_roll = [deg2rad(1)];
figure(3)
lsim(sys_roll,[],tspan,IC_roll);

%% Dutch-Roll
% x = [beta r]
A_dr = [Y_beta/u0 -(1-Y_r/u0);
        N_beta_prime N_r_prime];
sys_dr = ss(A_dr,[],eye(2),0);
damp(sys_dr)
IC_dr = [0; deg2rad(1)];
figure(4)
lsim(sys_dr,[],tspan,IC_dr);
omega_dr = sqrt((Y_beta*N_r_prime-N_beta_prime*Y_r+u0*N_beta_prime)/u0);
damp_dr = - 1/2/omega_dr*(Y_beta+u0*N_r_prime)/u0;