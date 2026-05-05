%% =========================================================
%  initial_param.m
%  BGRacing FSAE - Torque Vectoring Project
%  Tomer Tzahor & Itai Groisman, Ben-Gurion University
%
%  PURPOSE: Define all vehicle, motor, and controller
%           parameters. Run this script ONCE before
%           opening main_program.slx.
%% =========================================================

clear; clc;

%% --- Vehicle Parameters ---
Vehicle.Mass        = 240;                        % [kg]      Total mass (without driver)
Vehicle.L           = 1.54;                       % [m]       Wheelbase (front axle to rear axle)
Vehicle.TrackWidth  = 1.2;                        % [m]       *** Need to be filled ***
Vehicle.a           = 0.68;                       % [m]       Distance from C.G to front axle
Vehicle.b           = Vehicle.L - Vehicle.a;      % [m]       Distance from C.G to rear axle
Vehicle.Rw          = 0.235;                      % [m]       Wheel radius
Vehicle.Iz          = 245.9;                      % [kg*m^2]  Yaw moment of inertia

%% --- Tire Parameters (Cornering Stiffness) ---
Vehicle.Ca_front_wheel = 10000;                   % [N/rad]   Cornering stiffness per single front wheel
Vehicle.Ca_rear_wheel  = 9000;                    % [N/rad]   Cornering stiffness per single rear wheel
Vehicle.Cf          = 2 * Vehicle.Ca_front_wheel; % [N/rad]   Front axle total cornering stiffness
Vehicle.Cr          = 2 * Vehicle.Ca_rear_wheel;  % [N/rad]   Rear axle total cornering stiffness

%% --- Motor Parameters ---
Motor.MaxTorque     = 21;                         % [Nm]      Max torque per motor (at wheel shaft input)
Motor.GearRatio     = 11;                         % [-]       Transmission gear ratio
Motor.MaxWheelTorque = Motor.MaxTorque * Motor.GearRatio; % [Nm] Max torque at wheel

%% --- Physical Constants ---
Phys.g              = 9.81;                       % [m/s^2]   Gravitational acceleration
Phys.ay_max         = 1.0 * Phys.g;              % [m/s^2]   Max lateral acceleration (1g safety limit)

%% --- Understeer Gradient (derived from vehicle params) ---
% Kus > 0 => understeer (stable), Kus < 0 => oversteer (unstable)
% Kus = (m/L) * (b/Cf - a/Cr)
Vehicle.Kus = (Vehicle.Mass / Vehicle.L) * ...
              (Vehicle.b / Vehicle.Cf - Vehicle.a / Vehicle.Cr);

fprintf('Understeer Gradient Kus = %.6f [s^2/m]\n', Vehicle.Kus);
if Vehicle.Kus > 0
    fprintf('  -> Vehicle is UNDERSTEERING (stable baseline)\n');
elseif Vehicle.Kus < 0
    fprintf('  -> Vehicle is OVERSTEERING (unstable baseline - check params!)\n');
else
    fprintf('  -> Vehicle is NEUTRAL STEER\n');
end

%% --- Controller Settings ---
Control.Ts          = 0.001;                      % [s]       Sample time (1 kHz)
Control.MaxYawMoment = 1000;                      % [Nm]      Saturation limit on Mz output
Control.AntiWindup  = true;                       % [-]       Enable anti-windup on integrator

%% --- Torque Allocator Settings ---
% Strategy: distribute Mz equally between front and rear axles (50/50)
% Change Alloc.FrontBias to 0 for rear-only, 0.5 for equal front/rear, etc.
Alloc.FrontBias     = 0.5;                        % [-]       Fraction of Mz applied to front axle
Alloc.RearBias      = 1 - Alloc.FrontBias;        % [-]       Fraction of Mz applied to rear axle

%% --- Run Gain Scheduling Script ---
% This computes the Kp/Ki lookup table as a function of velocity
% and saves GainSchedule struct to workspace for Simulink
run('State_system.m');

fprintf('\ninitial_param.m complete. Workspace ready for Simulink.\n');
