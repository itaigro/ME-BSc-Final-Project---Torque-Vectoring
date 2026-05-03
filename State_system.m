%% =========================================================
%  State_system.m
%  BGRacing FSAE - Torque Vectoring Project
%  Tomer Tzahor & Itai Groisman, Ben-Gurion University
%
%  PURPOSE: Compute PI gain schedule (Kp, Ki) as a function
%           of longitudinal velocity using the linear 2-DOF
%           bicycle model. Saves lookup table vectors to
%           workspace for use in Simulink.
%
%  CALLED BY: initial_param.m (do not run standalone)
%% =========================================================

%% --- Design Targets ---
target_bandwidth    = 40;    % [rad/s]  Desired closed-loop bandwidth
target_pm           = 60.0;  % [deg]    Desired phase margin (~10% overshoot)

%% --- Velocity Range for Gain Scheduling ---
velocities = 3:2:25;         % [m/s]  From 3 m/s to 25 m/s in steps of 2

%% --- Pre-allocate results ---
n = length(velocities);
Kp_vec = zeros(1, n);
Ki_vec = zeros(1, n);
BW_vec = zeros(1, n);
PM_vec = zeros(1, n);

fprintf('------------------------------------------------------------\n');
fprintf(' Gain Scheduling: Computing PI gains across velocity range\n');
fprintf('------------------------------------------------------------\n');
fprintf(' v [m/s] |   Kp    |   Ki    |  BW [rad/s] | PM [deg]\n');
fprintf('---------|---------|---------|-------------|----------\n');

%% --- Loop over velocities ---
for i = 1:n
    v = velocities(i);

    % Build state-space matrices from Vehicle struct (set by initial_param.m)
    % States: x = [beta; r]  (sideslip angle [rad], yaw rate [rad/s])
    % Inputs: u = [delta; Mz] (steering angle [rad], yaw moment [Nm])

    a11 = -(Vehicle.Cf + Vehicle.Cr) / (Vehicle.Mass * v);
    a12 = (Vehicle.Cr * Vehicle.b - Vehicle.Cf * Vehicle.a) / (Vehicle.Mass * v^2) - 1;
    a21 = (Vehicle.Cr * Vehicle.b - Vehicle.Cf * Vehicle.a) / Vehicle.Iz;
    a22 = -(Vehicle.Cf * Vehicle.a^2 + Vehicle.Cr * Vehicle.b^2) / (Vehicle.Iz * v);

    A = [a11, a12;
         a21, a22];

    % B matrix: column 1 = steering input, column 2 = Mz (TV) input
    B = [Vehicle.Cf / (Vehicle.Mass * v),    0;
         Vehicle.Cf * Vehicle.a / Vehicle.Iz, 1 / Vehicle.Iz];

    C = [0, 1];   % Output: yaw rate only
    D = [0, 0];

    % Full 2-input system
    sys_full = ss(A, B, C, D);

    % Isolate the TV channel: Mz -> yaw rate (SISO plant for PI design)
    G_plant = sys_full(1, 2);

    % Design PI controller using pidtune
    opts = pidtuneOptions('PhaseMargin', target_pm);
    [C_pi, info] = pidtune(G_plant, 'PI', target_bandwidth, opts);

    % Store results
    Kp_vec(i) = C_pi.Kp;
    Ki_vec(i) = C_pi.Ki;
    BW_vec(i) = info.CrossoverFrequency;
    PM_vec(i) = info.PhaseMargin;

    fprintf('  %5.1f  | %7.4f | %7.4f |   %8.3f  | %7.2f\n', ...
        v, C_pi.Kp, C_pi.Ki, info.CrossoverFrequency, info.PhaseMargin);

    % Save full system at v=15 m/s for analysis plots
    if v == 15
        sys_v15   = G_plant;
        ctrl_v15  = C_pi;
    end
end

%% --- Save lookup table to workspace (used by Simulink 1D Lookup blocks) ---
GainSchedule.vel_vec = velocities;   % [m/s]  breakpoints
GainSchedule.Kp_vec  = Kp_vec;      % [Nm/(rad/s)]
GainSchedule.Ki_vec  = Ki_vec;      % [Nm/(rad)]

fprintf('\nGainSchedule struct saved to workspace.\n');
fprintf('  vel_vec: [%.1f ... %.1f] m/s (%d points)\n', ...
    velocities(1), velocities(end), n);

%% --- Summary Table ---
Results = table(velocities', Kp_vec', Ki_vec', BW_vec', PM_vec', ...
    'VariableNames', {'Velocity_ms','Kp','Ki','Bandwidth_rad_s','PhaseMargin_deg'});
disp(' ');
disp('=== Gain Schedule Table ===');
disp(Results);

%% --- Analysis Plots (at v = 15 m/s) ---
% Only plot if sys_v15 was set (i.e. 15 m/s is in the velocity range)
if exist('sys_v15', 'var')
    figure('Name', 'TV Controller Analysis (v = 15 m/s)', 'NumberTitle', 'off');

    % 1. Closed-loop step response
    subplot(2,2,1);
    sys_cl = feedback(ctrl_v15 * sys_v15, 1);
    step(sys_cl);
    grid on;
    title('Closed-Loop Step Response (v = 15 m/s)');
    xlabel('Time [s]'); ylabel('Yaw Rate [rad/s]');

    % 2. Open-loop Bode plot of plant
    subplot(2,2,2);
    bode(sys_v15);
    grid on;
    title('Plant Bode Plot: Mz \rightarrow r (v = 15 m/s)');

    % 3. Pole-zero map
    subplot(2,2,3);
    pzmap(sys_v15);
    grid on;
    title('Plant Pole-Zero Map (v = 15 m/s)');

    % 4. Root locus with PI controller
    subplot(2,2,4);
    rlocus(ctrl_v15 * sys_v15);
    grid on;
    title('Root Locus with PI Controller (v = 15 m/s)');
end

%% --- Gain Schedule Plot ---
figure('Name', 'Gain Schedule vs Velocity', 'NumberTitle', 'off');
subplot(2,1,1);
plot(velocities, Kp_vec, 'b-o', 'LineWidth', 1.5);
grid on;
xlabel('Velocity [m/s]'); ylabel('Kp');
title('Proportional Gain vs Velocity');

subplot(2,1,2);
plot(velocities, Ki_vec, 'r-o', 'LineWidth', 1.5);
grid on;
xlabel('Velocity [m/s]'); ylabel('Ki');
title('Integral Gain vs Velocity');
