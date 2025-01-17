%% Parameters
% mass and geometry
m = 1247.379; % mass, kg
J = diag([1420.89721,4067.45384,4786.03736]); % moment of inertia, kg*m^2
b = 10.18032; % wingspan, m
c = 1.73736; % wing mean aerodynamic chord, m
xR = (29.5 - 25)/100; % CG location from CP as a % of MAC
S = 17.0942; % wing area, m^2
AR = b^2/S; % wing aspect ratio
e = 0.98; % efficiency factor
g = 9.80665; % gravity, m/s^2

% stability and control derivatives
CD_0 = 0.05;
CD_alpha = 0.33;
CD_Mach = 0.0;
CL_0 = 0.41;
CL_alpha = 4.44;
CL_alpha_dot = 0.0;
CL_q = 3.8;
CL_Mach = 0.0;
CL_de = -0.923;
Cm_alpha = -0.683;
Cm_alpha_dot = -4.36;
Cm_q = -9.96;
Cm_Mach = 0.0;
Cm_de = -0.923;
Cy_beta = -0.564;
Cy_dr = 0.157;
Cl_beta = -0.074;
Cl_p = -0.41;
Cl_r = 0.107;
Cl_da = -0.134;
Cl_dr = 0.107;
Cn_beta = 0.071;
Cn_p = -0.0575;
Cn_r = -0.125;
Cn_da = -0.0035;
Cn_dr = -0.072;

Cx_dpt = 0.1161;
Cz_dpt = -0.1563;
Cm_dpt = -0.079;