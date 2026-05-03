%% =========================================================
%  torque_allocator.m
%  BGRacing FSAE - Torque Vectoring Project
%  Tomer Tzahor & Itai Groisman, Ben-Gurion University
%
%  PURPOSE: This is a MATLAB Function block to be placed
%           inside main_program.slx.
%
%  Copy this function body into a MATLAB Function block
%  in Simulink with:
%    Inputs:  Mz [Nm], T_driver [Nm] (base torque demand)
%    Outputs: T_FL, T_FR, T_RL, T_RR [Nm]
%% =========================================================

function [T_FL, T_FR, T_RL, T_RR] = torque_allocator(Mz, Gas_pedal)

%% --- Read parameters from base workspace ---
TrackWidth       = evalin('base', 'Vehicle.TrackWidth');   % [m]
Rw               = evalin('base', 'Vehicle.Rw');           % [m]
MaxWheelTorque   = evalin('base', 'Motor.MaxWheelTorque'); % [Nm]
FrontBias        = evalin('base', 'Alloc.FrontBias');      % [-]
RearBias         = evalin('base', 'Alloc.RearBias');       % [-]

%% --- Base torque demand from driver ---
% Gas_pedal is 0..1, scale to max wheel torque
T_driver = Gas_pedal * MaxWheelTorque;

%% --- Convert Mz to per-axle torque delta ---
% Mz = (ΔF_x) * (TrackWidth/2)
% ΔF_x = Mz / (TrackWidth/2)
% ΔT   = ΔF_x * Rw
delta_T_rear  = (Mz * RearBias)  * Rw / (TrackWidth / 2);
delta_T_front = (Mz * FrontBias) * Rw / (TrackWidth / 2);

%% --- Apply differential torque around base demand ---
% Positive Mz = yaw left = more torque on right wheels
T_RR = T_driver + delta_T_rear  / 2;
T_RL = T_driver - delta_T_rear  / 2;
T_FR = T_driver + delta_T_front / 2;
T_FL = T_driver - delta_T_front / 2;

%% --- Clamp to physical motor limits [0, MaxWheelTorque] ---
T_FL = max(0, min(T_FL, MaxWheelTorque));
T_FR = max(0, min(T_FR, MaxWheelTorque));
T_RL = max(0, min(T_RL, MaxWheelTorque));
T_RR = max(0, min(T_RR, MaxWheelTorque));

end
