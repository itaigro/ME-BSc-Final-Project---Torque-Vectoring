%% Vehicle Parameters - Formula Student Project
Vehicle.Mass        = 240;                       % [kg]      Mass of the car (without driver)
Vehicle.L           = 1.54;                      % [m]       Wheelbase
Vehicle.a           = 0.68;                       % [m]       Distance from C.G to front axle
Vehicle.b           = Vehicle.L - Vehicle.a;     % [m]       Distance from C.G to rear axle
Vehicle.Rw          = 0.235;                     % [m]       Wheel Radius
Vehicle.Iz          = 245.9;                       % [kg*m^2]  Yaw Moment of Inertia

%% Tire Parameters (Cornering Stiffness)
Vehicle.Ca_front_wheel = 10000;                   % [N/rad]   Cornering stiffness per SINGLE front wheel
Vehicle.Ca_rear_wheel  = 9000;                    % [N/rad]   Cornering stiffness per SINGLE rear wheel
Vehicle.Cf          = 2 * Vehicle.Ca_front_wheel; % [N/rad]   Front Axle Cornering Stiffness
Vehicle.Cr          = 2 * Vehicle.Ca_rear_wheel;  % [N/rad]   Rear Axle Cornering Stiffness

%% Motor Parameters
Motor.MaxTorque     = 21;                        % [Nm]      Max torque per motor
Motor.GearRatio     = 11;                        % [-]       Transmission ratio

%% Controller Constants
Control.Ts          = 0.001;                     % [s]       Sample time
Control.Kp = 1;    % [Nm]    Proportional gain for the controller
Control.Ki = 0.5;  % [Nms]   Integral gain for the controller
Control.MaxYawMoment = 1000; %maximum moment can be
%% Physical Constants
Phys.g              = 9.81;                      % [m/s^2]   Gravity acceleration

display((Vehicle.Mass/(Vehicle.a+Vehicle.b))*((Vehicle.b/Vehicle.Cf)-(Vehicle.a/Vehicle.Cr)))