%% Initial states
u_init = 54.194; % m/s
h_init = 1500; % m

pos = [0;0;-h_init]; % m
vel = [u_init;0;0]; % m/s
att = [deg2rad(0);deg2rad(0);deg2rad(0)]; % rad;
rate = [0;0;0]; % rad/s

wind_vel = [0;0;0]; % m/s
alpha_dot = 0;

da = 0; % rad
de = 0; % rad
dr = 0; % rad
n_rpm = idle_rpm; % idle RPM

rw_interal = 0; % rad/s
eI_theta = 0;
eI_u = 0;