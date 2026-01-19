clear; clc; close all
flightSim_createAircraft

%% Longitudinal-Related Aircraft Parameters
% general
g    = Environment.Gravity;
rho  = Environment.Density;
u0   = state.U;
Q    = 0.5*rho*u0^2;
Mach = u0/Environment.SpeedOfSound;
Jy   = state.Inertia(2,2).Variables;
m    = state.Mass;
S    = aircraft.ReferenceArea;
c    = aircraft.ReferenceLength;
b    = aircraft.ReferenceSpan;
alpha_0 = state.Alpha;
beta_0  = state.Beta;

% stability and control coefficients
CX_0      = getCoefficient(aircraft,"CX","Zero");
CX_alpha  = getCoefficient(aircraft,"CX","Alpha");
CX_alpha2 = getCoefficient(aircraft,"CX","Alpha2");
CX_alpha3 = getCoefficient(aircraft,"CX","Alpha3");
CX_q      = getCoefficient(aircraft,"CX","qcV");

CZ_0      = getCoefficient(aircraft,"CZ","Zero");
CZ_alpha  = getCoefficient(aircraft,"CZ","Alpha");
CZ_alpha3 = getCoefficient(aircraft,"CZ","Alpha3");
CZ_q      = getCoefficient(aircraft,"CZ","qcV");

Cm_0      = getCoefficient(aircraft,"Cm","Zero");
Cm_alpha  = getCoefficient(aircraft,"Cm","Alpha");
Cm_alpha2 = getCoefficient(aircraft,"Cm","Alpha2");
Cm_beta2  = getCoefficient(aircraft,"Cm","Beta2");
Cm_q      = getCoefficient(aircraft,"Cm","qcV");
Cm_r      = getCoefficient(aircraft,"Cm","rb2V");

CX_prop   = getCoefficient(aircraft,"CX","Propeller",Component="Propeller");
CX_dt     = max(diff(CX_prop.Table.Value)./diff(CX_prop.Breakpoints(1).Value));
CZ_de     = getCoefficient(aircraft,"CZ","Elevator",Component="Elevator");
CZ_deb2   = getCoefficient(aircraft,"CZ","ElevatorBeta2",Component="Elevator");
Cm_de     = getCoefficient(aircraft,"Cm","Elevator",Component="Elevator");

% stability and control derivatives
X_u       = Q*S/m/u0*CX_0*(2);
X_alpha   = Q*S/m*CX_alpha;
X_alpha2  = Q*S/m*CX_alpha2*(2*alpha_0);
X_alpha3  = Q*S/m*CX_alpha3*(3*alpha_0^2);
X_q       = Q*S/m/u0*CX_q*c;
X_dt      = Q*S/m*CX_dt;
X_de      = 0;

Z_u       = Q*S/m/u0*CZ_0*(2);
Z_alpha   = Q*S/m*CZ_alpha;
Z_alpha3  = Q*S/m*CZ_alpha3*(3*alpha_0^2);
Z_q       = Q*S/m/u0*CZ_q*c;
Z_dt      = 0;
Z_de      = Q*S/m*CZ_de;
Z_deb2    = Q*S/m*CZ_deb2*(2*beta_0);

M_u       = Q*S*c/Jy/u0*Cm_0*(2);
M_alpha   = Q*S*c/Jy*Cm_alpha;
M_alpha2  = Q*S*c/Jy*Cm_alpha2*(2*alpha_0);
M_beta2   = Q*S*c/Jy*Cm_beta2*(2*beta_0);
M_q       = Q*S*c/Jy/u0*Cm_q*c;
M_r       = Q*S*c/Jy/u0/2*Cm_r*b;
M_dt      = 0;
M_de      = Q*S*c/Jy*Cm_de;

%% Longitudinal Dynamics
% x = [u; alpha; q; theta]
A = [X_u    X_alpha     0 -g;
     Z_u/u0 Z_alpha/u0  1  0;
     M_u    M_alpha    M_q 0;
      0        0        1  0];
B = [X_de     X_dt;
     Z_de/u0  Z_dt;
     M_de     M_dt;
     0         0];
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
A_sp = [Z_alpha/u0 1; M_alpha M_q];
sys_sp = ss(A_sp,[],eye(2),0);
damp(sys_sp)
IC_sp = [0;deg2rad(1)];
figure(3)
lsim(sys_sp,[],tspan,IC_sp);
omega_sp = sqrt(-Z_alpha*M_q/u0 - M_alpha);
damp_sp = -(M_q + Z_alpha/u0)/(2*omega_sp);