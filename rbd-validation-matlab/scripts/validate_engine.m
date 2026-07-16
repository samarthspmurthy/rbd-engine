% validate_engine.m
clear; clc; close all;

%% STEP 1: DATA INGESTION
% Using the absolute paths to locate the files across different folders
cpp_file = 'C:\Users\91953\Desktop\rbd-engine\data\output\simulation_log.csv'; 
sim_file = 'C:\Users\91953\Desktop\rbd-engine\rbd-validation-matlab\simscape_log.csv';

% Read pure numerical data (bypassing header strings)
% Starting from row 2 to skip the header text
cpp_raw = readmatrix(cpp_file, 'NumHeaderLines', 1);
sim_raw = readmatrix(sim_file, 'NumHeaderLines', 1);

% Extract C++ Core States (Based on visual index mapping)
% Col 1=time, Col 2=th1, Col 3=th2, Col 4=th1_dot, Col 5=th2_dot
t_cpp       = cpp_raw(:, 1);
th1_cpp     = cpp_raw(:, 2);
th2_cpp     = cpp_raw(:, 3);
th1_dot_cpp = cpp_raw(:, 4);
th2_dot_cpp = cpp_raw(:, 5);

% Extract Simscape Core States
% Col 1=time, Col 2=th1, Col 3=th2, Col 4=th1_dot, Col 5=th2_dot
t_sim       = sim_raw(:, 1);
th1_sim     = sim_raw(:, 2);
th2_sim     = sim_raw(:, 3);
th1_dot_sim = sim_raw(:, 4);
th2_dot_sim = sim_raw(:, 5);

%% STEP 2: CORE STATE VALIDATION & ERROR METRICS

% Safety check: Ensure arrays are the same length before doing math to avoid
% concatenation or dimension mismatch errors.
min_len = min(length(t_cpp), length(t_sim));

t_err   = t_cpp(1:min_len);
err_th1 = abs(th1_cpp(1:min_len) - th1_sim(1:min_len));
err_th2 = abs(th2_cpp(1:min_len) - th2_sim(1:min_len));

% Calculate Error Metrics
max_err_th1 = max(err_th1);
max_err_th2 = max(err_th2);
rmse_th1    = sqrt(mean(err_th1.^2));
rmse_th2    = sqrt(mean(err_th2.^2));

fprintf('--- Validation Metrics (Radians) ---\n');
fprintf('Theta 1 | Max Error: %e | RMSE: %e\n', max_err_th1, rmse_th1);
fprintf('Theta 2 | Max Error: %e | RMSE: %e\n', max_err_th2, rmse_th2);

% --- Plotting: State Overlays ---
figure('Name', 'Core State Overlay', 'Position', [100, 100, 1000, 600]);

subplot(2,1,1);
plot(t_cpp, th1_cpp, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, th1_sim, 'r--', 'LineWidth', 1.5);
title('\theta_1 vs Time'); xlabel('Time (s)'); ylabel('\theta_1 (rad)');
legend('C++ RK4', 'Simscape ode4', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t_cpp, th2_cpp, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, th2_sim, 'r--', 'LineWidth', 1.5);
title('\theta_2 vs Time'); xlabel('Time (s)'); ylabel('\theta_2 (rad)');
legend('C++ RK4', 'Simscape ode4', 'Location', 'best');
grid on;

% --- Plotting: Error Over Time ---
figure('Name', 'Absolute Error Over Time', 'Position', [150, 150, 800, 400]);
semilogy(t_err, err_th1, 'b', 'LineWidth', 1.5); hold on;
semilogy(t_err, err_th2, 'r', 'LineWidth', 1.5);
title('Absolute Error over Time (Log Scale)');
xlabel('Time (s)'); ylabel('Absolute Error (rad)');
legend('Error \theta_1', 'Error \theta_2', 'Location', 'best');
grid on;

%% STEP 3: FORWARD KINEMATICS VALIDATION

% --- Load Physical Parameters ---
set_common_params; 

% --- Extract C++ Cartesian States ---
% CORRECTED INDICES based on actual raw CSV values
elbow_x_cpp = cpp_raw(:, 6);
elbow_y_cpp = cpp_raw(:, 7);
ee_x_cpp    = cpp_raw(:, 8);
ee_y_cpp    = cpp_raw(:, 9);

% --- Calculate Simscape Cartesian States (The Math) ---
% Using relative joint angles matching the C++ FK
elbow_x_sim = l1 .* cos(th1_sim);
elbow_y_sim = l1 .* sin(th1_sim);
ee_x_sim    = elbow_x_sim + l2 .* cos(th1_sim + th2_sim);
ee_y_sim    = elbow_y_sim + l2 .* sin(th1_sim + th2_sim);

% --- Calculate Cartesian Errors ---
err_ee_x = abs(ee_x_cpp(1:min_len) - ee_x_sim(1:min_len));
err_ee_y = abs(ee_y_cpp(1:min_len) - ee_y_sim(1:min_len));

fprintf('\n--- Forward Kinematics Metrics (Meters) ---\n');
fprintf('End-Effector X | Max Error: %e\n', max(err_ee_x));
fprintf('End-Effector Y | Max Error: %e\n', max(err_ee_y));

% --- Plotting: End-Effector Overlay ---
figure('Name', 'End-Effector Cartesian Overlay', 'Position', [200, 200, 1000, 600]);

subplot(2,1,1);
plot(t_cpp, ee_x_cpp, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, ee_x_sim, 'r--', 'LineWidth', 1.5);
title('End-Effector X Position vs Time'); xlabel('Time (s)'); ylabel('X (m)');
legend('C++ Engine', 'Simscape Derived', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t_cpp, ee_y_cpp, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, ee_y_sim, 'r--', 'LineWidth', 1.5);
title('End-Effector Y Position vs Time'); xlabel('Time (s)'); ylabel('Y (m)');
legend('C++ Engine', 'Simscape Derived', 'Location', 'best');
grid on;