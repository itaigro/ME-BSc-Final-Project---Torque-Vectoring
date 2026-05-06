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
Vehicle.Rw          = 0.23876;                      % [m]       Wheel radius
Vehicle.Rw_eff      = Vehicle.Rw*0.98;            % effective wheel radius [m]
Vehicle.Iz          = 245.9;                      % [kg*m^2]  Yaw moment of inertia

%% --- Sensors Parameters ---
Speed.tau_gps       = 0.3;
Speed.tau_RPM       = 0.05;
Speed.alpha         =0.95;
%% --- Tire Parameters (Cornering Stiffness) ---
Vehicle.Ca_front_wheel = 10000;                   % [N/rad]   Cornering stiffness per single front wheel
Vehicle.Ca_rear_wheel  = 9000;                    % [N/rad]   Cornering stiffness per single rear wheel
Vehicle.Cf          = 2 * Vehicle.Ca_front_wheel; % [N/rad]   Front axle total cornering stiffness
Vehicle.Cr          = 2 * Vehicle.Ca_rear_wheel;  % [N/rad]   Rear axle total cornering stiffness

%% --- Motor Parameters ---
Motor.MaxTorque     = 21;                         % [Nm]      Max torque per motor (at wheel shaft input)
Motor.GearRatio     = 11;                         % [-]       Transmission gear ratio
Motor.MaxWheelTorque = Motor.MaxTorque * Motor.GearRatio; % [Nm] Max torque at wheel
Motor.RPM_axis = [0, 2006, 4013, 6019, 8008, 9960, 12003, 14027, 15998, 17986, 18999]; %[RPM -calculated from orange line in the motor graph]
Motor.Torque_axis = [13.8, 12.6, 11.9, 11.5, 10.8, 10.2, 9.8, 9.2, 8.9, 8.3, 0.2]; % [Nm]
Motor.RPM_axis_blue = [0, 2006, 7990, 12979, 14009, 16016, 17661, 18023, 18999];
Motor.Torque_axis_blue = [21.2, 21.2, 21.2, 21.2, 19.7, 15.5, 10, 8.5, 0.3];

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

%% --- Offline Gain Scheduling Calculation ---
Control.vel_breakpoints = 3:1:25; 
n = length(Control.vel_breakpoints);

Control.Kp_table = zeros(1,n);
Control.Ki_table = zeros(1,n);

fprintf('\n========================================================\n');
fprintf('   Torque Vectoring Gain Schedule: PI Tuning (RAD/SEC)  \n');
fprintf('========================================================\n');
fprintf(' Velocity [m/s] |    Kp [Nm/(rad/s)]   |   Ki [Nm/rad]  \n');
fprintf('----------------|----------------------|----------------\n');

for i = 1:n
    v_curr = Control.vel_breakpoints(i);
    
    % A matrix from mid-year report (Eq. 4.17)
    a11 = -(Vehicle.Cf + Vehicle.Cr) / (Vehicle.Mass * v_curr);
    a12 = (Vehicle.Cr * Vehicle.b - Vehicle.Cf * Vehicle.a) / (Vehicle.Mass * v_curr^2) - 1;
    a21 = (Vehicle.Cr * Vehicle.b - Vehicle.Cf * Vehicle.a) / Vehicle.Iz;
    a22 = -(Vehicle.Cf * Vehicle.a^2 + Vehicle.Cr * Vehicle.b^2) / (Vehicle.Iz * v_curr);
    A = [a11, a12; a21, a22];
    
    % B matrix (Column 2: Mz effect, Eq. 4.17)
    B = [Vehicle.Cf / (Vehicle.Mass * v_curr), 0;
         Vehicle.Cf * Vehicle.a / Vehicle.Iz,  1/Vehicle.Iz];
    
    C = [0, 1]; % Output: Yaw Rate [rad/s]
    D = [0, 0];
    
    sys_rad = ss(A, B, C, D);
    G_plant = sys_rad(1, 2); 
    
    % Tune PI controller for the RAD/SEC plant
    opts = pidtuneOptions('PhaseMargin', 60);
    C_pi = pidtune(G_plant, 'PI', 40, opts);
    
    Control.Kp_table(i) = C_pi.Kp;
    Control.Ki_table(i) = C_pi.Ki;
    
    fprintf('      %4.1f      |      %12.4f    |    %12.4f  \n', ...
            v_curr, Control.Kp_table(i), Control.Ki_table(i));
end
%% --- Torque Allocator Settings ---
% Strategy: distribute Mz equally between front and rear axles (50/50)
% Change Alloc.FrontBias to 0 for rear-only, 0.5 for equal front/rear, etc.
Alloc.FrontBias     = 0.5;                        % [-]       Fraction of Mz applied to front axle
Alloc.RearBias      = 1 - Alloc.FrontBias;        % [-]       Fraction of Mz applied to rear axle

%% --- Run Gain Scheduling Script ---
% This computes the Kp/Ki lookup table as a function of velocity
% and saves GainSchedule struct to workspace for Simulink
%
%  run('State_system.m');
% fprintf('\ninitial_param.m complete. Workspace ready for Simulink.\n');
