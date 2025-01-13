% add this file to Simulink workspace and set a fixed simulation timestep
clear; clc; close all

% mass and geometry
params.m = 1247.379; % mass, kg
params.J = diag([1420.89721,4067.45384,4786.03736]); % moment of inertia, kg*m^2
params.b = 10.18032; % wingspan, m
params.c = 1.73736; % wing mean aerodynamic chord, m
params.xR = (29.5 - 25)/100; % CG location from CP as a % of MAC
params.S = 17.0942; % wing area, m^2
params.AR = params.b^2/params.S; % wing aspect ratio
params.e = 0.98; % efficiency factor
params.g = 9.80665; % gravity, m/s^2

% stability and control derivatives
params.CD0 = 0.05;
params.CDa = 0.33;
params.CDM = 0.0;
params.CL0 = 0.41;
params.CLa = 4.44;
params.CLa_dot = 0.0;
params.CLq = 3.8;
params.CLM = 0.0;
params.CLde = -0.923;
params.Cma = -0.683;
params.Cma_dot = -4.36;
params.Cmq = -9.96;
params.CmM = 0.0;
params.Cmde = -0.923;
params.Cyb = -0.564;
params.Cydr = 0.157;
params.Clb = -0.074;
params.Clp = -0.41;
params.Clr = 0.107;
params.Clda = -0.134;
params.Cldr = 0.107;
params.Cnb = -0.071;
params.Cnp = -0.0575;
params.Cnr = -0.125;
params.Cnda = -0.0035;
params.Cndr = -0.072;

% initial states
u0 = 54.194; % m/s
z0 = -1500; % m
params.P_max = 190000; % W
