clear; clc; close all
AircraftParameters
AircraftInitialization

%% Lateral
rho = 1.225;
u0 = vel(1);
Q = 0.5*rho*u0^2;
Mach = 0.158;
Jx = J(1,1);
Jz = J(3,3);
Jxz = J(1,3);

Y_beta = Q*S/m*Cy_beta;
Y_p = 0;
Y_r = 0;
Y_dr = Q*S/m*Cy_dr;
Y_da = 0;
L_beta = Q*S*b/Jx*Cl_beta;
L_p = Q*S*b/Jx*b/2/u0*Cl_p;
L_r = Q*S*b/Jx*b/2/u0*Cl_r;
L_da = Q*S*b/Jx*Cl_da;
L_dr = Q*S*b/Jx*Cl_dr;
N_beta = Q*S*b/Jz*Cn_beta;
N_p = Q*S*b/Jz*b/2/u0*Cn_p;
N_r = Q*S*b/Jz*b/2/u0*Cn_r;
N_da = Q*S*b/Jz*Cn_da;
N_dr = Q*S*b/Jz*Cn_dr;

Gamma = 1 - Jxz^2/Jx/Jz;
L_beta_prime = L_beta/Gamma + Jxz/Jx*N_beta;
L_p_prime = L_p/Gamma + Jxz/Jx*N_p;
L_r_prime = L_r/Gamma + Jxz/Jx*N_r;
N_beta_prime = N_beta/Gamma + Jxz/Jz*L_beta;
N_p_prime = N_p/Gamma + Jxz/Jz*L_p;
N_r_prime = N_r/Gamma + Jxz/Jz*L_r;

% x = [beta; p; r; phi]
A = [Y_beta/u0 Y_p/u0 -(1-Y_r/u0) g/u0;
     L_beta_prime L_p_prime L_r_prime 0;
     N_beta_prime N_p_prime N_r_prime 0;
     0 1 0 0];
B = [Y_da/u0 Y_dr/u0;
     L_da L_dr;
     N_da N_dr;
     0 0];

sys = ss(A,[],eye(4),0);
damp(sys)

tspan = 0:0.01:50;
IC = [deg2rad(1);deg2rad(1);deg2rad(1);deg2rad(0)];
figure(1)
lsim(sys,[],tspan,IC);

%% Spiral
A_spi = N_r_prime - L_r_prime*N_beta_prime/L_beta_prime;
sys_spi = ss(A_spi,[],eye(1),0);
damp(sys_spi)
IC_spi = [deg2rad(1)];
figure(2)
lsim(sys_spi,[],tspan,IC_spi);

%% Roll
A_roll = L_p_prime;
sys_roll = ss(A_roll,[],eye(1),0);
damp(sys_roll)
IC_roll = [deg2rad(1)];
figure(3)
lsim(sys_roll,[],tspan,IC_roll);

%% Dutch-Roll
A_dr = [Y_beta/u0 -(1-Y_r/u0);
        N_beta_prime N_r_prime];
sys_dr = ss(A_dr,[],eye(2),0);
damp(sys_dr)
IC_dr = [deg2rad(1)];
figure(4)
lsim(sys_dr,[],tspan,IC_dr);
omega_dr = sqrt((Y_beta*N_r_prime-N_beta_prime*Y_r+u0*N_beta_prime)/u0);
damp_dr = - 1/2/omega_dr*(Y_beta+u0*N_r_prime)/u0;

% T_half = 0.69/abs(real)
% Period = 2*pi/abs(img)
% N_half = T_half/P = 0.11*abs(img)/abs(real)