%% FSAE Torque Vectoring Controller Design - Master Script
% Project: Formula Student Electric - Torque Vectoring
% Author: Tomer & Itai (Assisted by AI)
% Based on Linear Single Track Model (Mondek Thesis)

clear; clc; close all;

%% 1. הגדרת פרמטרים של הרכב (נא לעדכן למספרים שלכם!)
% ---------------------------------------------------------
m  = 280;      % מסת הרכב [kg] (כולל נהג משוער)
Iz = 250;      % מומנט אינרציה סביב ציר Z [kg*m^2]
lf = 0.78;     % מרחק מרכז כובד לציר קדמי [m]
lr = 0.75;     % מרחק מרכז כובד לציר אחורי [m]
Cf = 22500;    % קשיחות פנייה צמיג קדמי (N/rad) - מוערך
Cr = 25000;    % קשיחות פנייה צמיג אחורי (N/rad) - מוערך

% דרישות ביצועים לבקר (Tuning Goals)
target_bandwidth = 40;  % רוחב סרט רצוי [rad/s] - קובע את מהירות התגובה
target_pm = 60.0;         % עודף פאזה רצוי [deg] - קובע את היציבות (כ-10% Overshoot)

%% 2. לולאת חישוב (Gain Scheduling) למהירויות שונות
% נחשב את המקדמים עבור טווח מהירויות כדי לראות איך הם משתנים

velocities = 5:5:25; % בדיקה מ-5 מ'/ש' עד 25 מ'/ש'
results = table();   % טבלה לשמירת התוצאות

fprintf('------------------------------------------------------------\n');
fprintf('Calculating PI Gains for different velocities:\n');
fprintf('------------------------------------------------------------\n');

for i = 1:length(velocities)
    v = velocities(i);
    
    % --- בניית מטריצות מרחב המצב (State Space) ---
    % States: x = [beta; r] (Side slip; Yaw rate)
    % Inputs: u = [delta; Mz] (Steering; Torque Moment)
    
    a11 = -(Cf + Cr) / (m * v);
    a12 = -1 - (Cf * lf - Cr * lr) / (m * v^2);
    a21 = -(Cf * lf - Cr * lr) / Iz;
    a22 = -(Cf * lf^2 + Cr * lr^2) / (Iz * v);
    
    A = [a11, a12; 
         a21, a22];
     
    % B Matrix - שתי עמודות!
    % עמודה 1: היגוי (delta)
    b1_steer = Cf / (m * v);
    b2_steer = (Cf * lf) / Iz;
    
    % עמודה 2: מומנט (Mz) - הכניסה של ה-Torque Vectoring
    b1_moment = 0;      % מומנט לא מייצר כוח צד ישיר
    b2_moment = 1 / Iz; % מומנט מייצר תאוצת סבסוב ישירה
    
    B = [b1_steer, b1_moment; 
         b2_steer, b2_moment];
     
    C = [0, 1]; % אנחנו מודדים רק Yaw Rate
    D = [0, 0]; 
    
    % יצירת מערכת מלאה (2 כניסות)
    sys_full = ss(A, B, C, D);
    sys_full.InputName = {'Steering', 'Mz'};
    sys_full.OutputName = {'YawRate'};
    
    % --- בידוד ערוץ המומנט (SISO Plant) לתכנון הבקר ---
    % אנו רוצים לשלוט ב-YawRate באמצעות Mz בלבד
    G_plant = sys_full(1, 2); 
    
    % --- תכנון הבקר (PID Tuning) ---
    opts = pidtuneOptions('PhaseMargin', target_pm);
    [C_pi, info] = pidtune(G_plant, 'PI', target_bandwidth, opts);
    
    % שמירת התוצאות לטבלה
    newRow = {v, C_pi.Kp, C_pi.Ki, info.CrossoverFrequency, info.PhaseMargin};
    results = [results; newRow];
    
    % הצגת פונקציית התמסורת עבור מהירות נומינלית (למשל 15 מ'/ש') לטובת הדו"ח
    if v == 15
        sys_design_v15 = G_plant;
        ctrl_design_v15 = C_pi;
        fprintf('\n>> Design Point (v = 15 m/s):\n');
        disp('Transfer Function G(s) = YawRate(s) / Mz(s):');
        tf(G_plant)
    end
end

% מתן שמות לעמודות הטבלה
results.Properties.VariableNames = {'Velocity_ms', 'Kp', 'Ki', 'Bandwidth_rad_s', 'PhaseMargin_deg'};

%% 3. הצגת תוצאות מסכמות
disp(' ');
disp('=== Final Calculated Gains Table ===');
disp(results);
disp('Use these Kp and Ki values in your Simulink Look-up Table based on velocity.');

%% 4. שרטוט גרפים לניתוח (עבור המהירות האחרונה שחושבה)
figure('Name', 'Control System Analysis');

% א. תגובת מדרגה (Step Response) בחוג סגור
subplot(2,2,1);
sys_cl = feedback(ctrl_design_v15 * sys_design_v15, 1);
step(sys_cl);
grid on;
title('Closed Loop Step Response (v=15m/s)');
ylabel('Yaw Rate Tracking');

% ב. דיאגרמת בודה (Bode Plot) - חוג פתוח
subplot(2,2,2);
bode(sys_design_v15);
grid on;
title('Plant Bode Plot (Mz -> r)');

% ג. מפת קטבים ואפסים (Pole-Zero Map)
subplot(2,2,3);
pzmap(sys_design_v15);
grid on;
title('Plant Pole-Zero Map');

% ד. Root Locus (מקום שורשים של המערכת המבוקרת)
subplot(2,2,4);
rlocus(ctrl_design_v15 * sys_design_v15);
grid on;
title('Root Locus with PI Controller');