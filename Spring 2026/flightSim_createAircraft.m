clear; clc; close all

%% Aircraft
aileron = Aero.FixedWing.Surface(1, "Controllable", "on", ...
    "MaximumValue",deg2rad(30),"MinimumValue", deg2rad(-30),...
    "Properties", Aero.Aircraft.Properties(1, "Name", "Aileron"), ...
    "Coefficients", Aero.FixedWing.Coefficient("StateVariables", ["Aileron", "AileronAlpha"], ...
    "ReferenceFrame", "Body"));

flap = Aero.FixedWing.Surface(1, "Controllable", "on", ...
    "MaximumValue",deg2rad(30),"MinimumValue", deg2rad(-30),...
    "Properties", Aero.Aircraft.Properties(1, "Name", "Flap"), ...
    "Coefficients", Aero.FixedWing.Coefficient("StateVariables", ["Flap", "FlapAlpha"], ...
    "ReferenceFrame", "Body"));

elevator = Aero.FixedWing.Surface(1, "Controllable", "on", ...
    "MaximumValue",deg2rad(30),"MinimumValue", deg2rad(-30),...
    "Properties", Aero.Aircraft.Properties(1, "Name", "Elevator"), ...
    "Coefficients", Aero.FixedWing.Coefficient("StateVariables", ["Elevator", "ElevatorBeta2"], ...
    "ReferenceFrame", "Body"));

rudder = Aero.FixedWing.Surface(1, "Controllable", "on", ...
    "MaximumValue",deg2rad(60),"MinimumValue", deg2rad(-60),...
    "Properties", Aero.Aircraft.Properties(1, "Name", "Rudder"), ...
    "Coefficients", Aero.FixedWing.Coefficient("StateVariables", ["Rudder", "RudderAlpha"], ...
    "ReferenceFrame", "Body"));

wing = Aero.FixedWing.Surface(1, "Surfaces", [aileron, flap], ...
    "Properties", Aero.Aircraft.Properties(1, "Name", "Wing"));

horizontalStabilizer = Aero.FixedWing.Surface(1, "Surfaces", elevator, ...
    "Properties", Aero.Aircraft.Properties(1, "Name", "HorizontalStabilizer"));

verticalStabilizer = Aero.FixedWing.Surface(1, "Surfaces", rudder, ...
    "Properties", Aero.Aircraft.Properties(1, "Name", "VerticalStabilizer"));

propeller = Aero.FixedWing.Thrust(1, "Properties", ...
    Aero.Aircraft.Properties(1, "Name","Propeller"), ...
    "Coefficients", Aero.FixedWing.Coefficient("StateVariables", "Propeller", ...
    "ReferenceFrame", "Body", "MultiplyStateVariable", "off", "NonDimensional", false));

name = Aero.Aircraft.Properties(1, ...
    "Name", "Dehavilland_Beaver", ...
    "Type", "General Aviation", ...
    "Version","1.0", ...
    "Description","Dehavilland Beaver demo");

aircraft = Aero.FixedWing(1, ...
    "Properties",name, ...
    "ReferenceArea", 23.2300, ...
    "ReferenceSpan", 14.6300, ...
    "ReferenceLength", 1.5875, ...
    "Surfaces",[wing, horizontalStabilizer, verticalStabilizer], ...
    "Thrusts",propeller);

% Coefficients: Zero, U, Alpha, AlphaDot, Beta, BetaDot, P, Q, R are created by default.
aircraft.Coefficients.ReferenceFrame = "Body";
aircraft.Coefficients.StateVariables = [aircraft.Coefficients.StateVariables, "Alpha2", "Alpha3", "Beta2", "Beta3", "qcV", "pb2V", "rb2V"];

BodyCoefficients = {
    'Cl', 'Zero', 5.9100e-04;
    'Cl', 'Beta', -0.0618;
    'Cl', 'pb2V', -0.5045;
    'Cl', 'rb2V', 0.1695;
    'Cm', 'Zero', 0.0945;
    'Cm', 'Alpha', -0.6028;
    'Cm', 'Alpha2', -2.1400;
    'Cm', 'Beta2', 0.6921;    
    'Cm', 'qcV', -15.5600;
    'Cm', 'rb2V', -0.3118;
    'Cn', 'Zero', -0.0031;
    'Cn', 'Beta', 0.0067;
    'Cn', 'Beta3', 0.1373;
    'Cn', 'pb2V', -0.1585;
    'Cn', 'qcV', 0.1595;
    'Cn', 'rb2V', -0.1112;
    'CX', 'Zero', -0.0355;
    'CX', 'Alpha', 0.0029;
    'CX', 'Alpha2', 5.4590;
    'CX', 'Alpha3', -5.1620;
    'CX', 'qcV', -0.6748;
    'CY', 'Zero', -0.0022;
    'CY', 'Beta', -0.7678;
    'CY', 'pb2V', -0.1240;
    'CY', 'rb2V', 0.3666;
    'CZ', 'Zero', -0.0550;
    'CZ', 'Alpha', -5.5780;
    'CZ', 'Alpha3', 3.4420;
    'CZ', 'qcV', -2.9880;
    };

ElevatorCoefficients = {
    'CZ', 'Elevator', -0.3980;
    'CZ', 'ElevatorBeta2', -15.9300;
    'Cm', 'Elevator', -1.9210;
    };

AileronCoefficients = {
    'Cl', 'Aileron', -0.0992;
    'Cl', 'AileronAlpha', -0.0827;
    'Cn', 'Aileron', -0.0039;
    'CY', 'Aileron', -0.0296;
    };

FlapCoefficients = {
    'CX', 'Flap', -0.0945;
    'CX', 'FlapAlpha', 1.1060;
    'CZ', 'Flap', -1.3770;
    'CZ', 'FlapAlpha', -1.2610;
    'Cm', 'Flap', 0.4072;
    };

RudderCoefficients = {
    'CX', 'Rudder', 0.0341;
    'Cn', 'Rudder', -0.0827;
    'Cl', 'Rudder', 0.0069;
    'CY', 'RudderAlpha', 0.5238;
    'CY', 'Rudder', 0.1158;
    };

% thrust curve
Thrust = 2300;
prop = Simulink.LookupTable;
prop.Table.Value = Thrust*(1+tanh(-3:3));
prop.Breakpoints(1).Value = (1/6)*(3+(-3:3));
prop.Breakpoints(1).FieldName = "Propeller";
        
PropellerCoefficients = {
    'CX', 'Propeller', prop;
    };

aircraft = setCoefficient(aircraft, BodyCoefficients(:, 1), BodyCoefficients(:, 2), BodyCoefficients(:, 3));
aircraft = setCoefficient(aircraft, AileronCoefficients(:, 1), AileronCoefficients(:, 2), AileronCoefficients(:, 3), "Component", "Aileron");
aircraft = setCoefficient(aircraft, FlapCoefficients(:, 1), FlapCoefficients(:, 2), FlapCoefficients(:, 3), "Component", "Flap");
aircraft = setCoefficient(aircraft, ElevatorCoefficients(:, 1), ElevatorCoefficients(:, 2), ElevatorCoefficients(:, 3), "Component", "Elevator");
aircraft = setCoefficient(aircraft, RudderCoefficients(:, 1), RudderCoefficients(:, 2), RudderCoefficients(:, 3), "Component", "Rudder");
aircraft = setCoefficient(aircraft, PropellerCoefficients(:, 1), PropellerCoefficients(:, 2), PropellerCoefficients(:, 3), "Component", "Propeller");

aircraft = aircraft.update(); % update cofficients to the new assigned values
thrustX = getCoefficient(aircraft,"CX","Propeller",Component="Propeller");

AltitudeMSL = 2202; % operation height
Environment = aircraftEnvironment(aircraft,"COESA",AltitudeMSL); % rho, P, T, a

state = fixedWingState(aircraft, Environment);
state.Mass              = 2288.231; % mass [kg]
state.Inertia.Variables = [ 5788.0     0.0  -117.6; 
                               0.0  6928.9     0.0;
                            -117.6     0.0 11578.3]; % Moment of Inertia [kg*m^2]
state.CenterOfGravity  = [0 0 0];     % Center of Gravity [m]
state.CenterOfPressure = [0 0 0];     % Center of Pressure [m]

% set initial condition (pick one)
[state, control] = spiralInitialCondition(state, AltitudeMSL);
% [state, control] = cruiseInitialCondition(state, AltitudeMSL);
disp('Aircraft Created')

%% Initial States
function [state_out, control] = spiralInitialCondition(state_in, AltitudeMSL)
    state_out = state_in;
    state_out.XN    = 0;            % Initial x position [m]
    state_out.XE    = 0;            % Initial y position [m]
    state_out.XD    = -AltitudeMSL; % Initial z position [m]
    state_out.U     = 44.54;        % Initial x velocity [m/s]
    state_out.V     = 2.714;        % Initial y velocity [m/s]
    state_out.W     = 5.836;        % Initial z velocity [m/s]
    state_out.Phi   = 0.0;          % Initial x angular position [rad]
    state_out.Theta = 0.1309;       % Initial y angular position [rad]
    state_out.Psi   = 0.0;          % Initial z angular position [rad]
    state_out.P     = 0.0;          % Initial x angular velocity [rad/s]
    state_out.Q     = 0.0;          % Initial y angular velocity [rad/s]
    state_out.R     = 0.0;          % Initial z angular velocity [rad/s]

    control.Throttle  = 0.5; % Initial Throttle Setting [0-1]
    control.Aileron   = 0.0; % Initial Aileron Deflection [rad]
    control.Elevator  = 0.0; % Initial Elevator Deflection [rad]
    control.Rudder    = 0.0; % Initial Rudder Deflection [rad]
end

function [state_out, control] = cruiseInitialCondition(state_in, AltitudeMSL)
    state_out = state_in;
    state_out.XN    = 0;                        % Initial x position [m]
    state_out.XE    = 0;                        % Initial y position [m]
    state_out.XD    = -AltitudeMSL;             % Initial z position [m]
    state_out.U     = 36.911969266574350;       % Initial x velocity [m/s]
    state_out.V     = 8.107208632644118;        % Initial y velocity [m/s]
    state_out.W     = 8.825918825964060;        % Initial z velocity [m/s]
    state_out.Phi   = 0.128665353088755;        % Initial x angular position [rad]
    state_out.Theta = 0.259337659945870;        % Initial y angular position [rad]
    state_out.Psi   = -0.178947917154394;       % Initial z angular position [rad]
    state_out.P     = 0.0;                      % Initial x angular velocity [rad/s]
    state_out.Q     = 0.0;                      % Initial y angular velocity [rad/s]
    state_out.R     = 0.0;                      % Initial z angular velocity [rad/s]

    control.Throttle  = 0.5;                    % Initial Throttle Setting [0-1]
    control.Aileron   = -0.104711430293605;     % Initial Aileron Deflection [rad]
    control.Elevator  = -0.069872522954441;     % Initial Elevator Deflection [rad]
    control.Rudder    = 0;%-2.150423302540860e-04; % Initial Rudder Deflection [rad]
end