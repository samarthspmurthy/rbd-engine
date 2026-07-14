% export_validation_data.m
% Step 5 & 6: Format, calculate velocity, and export Simscape data

disp('Structuring simulation data...');

% 1. Extract time and positions directly from the 5001x1 arrays
sim_time = out.tout;
theta1   = out.simout;   % Column 1 was position
theta2   = out.simout1;  % Column 1 was position

% 2. Numerically calculate velocities using the precise time steps
theta1_dot = gradient(theta1, sim_time);
theta2_dot = gradient(theta2, sim_time);

% 3. Package everything into the exact C++ contract layout table
validation_table = table(sim_time, theta1, theta2, theta1_dot, theta2_dot, ...
    'VariableNames', {'Time', 'Theta1', 'Theta2', 'Theta1_dot', 'Theta2_dot'});

% Step 6: Export to CSV
csv_filename = 'simscape_reference_data.csv';
writetable(validation_table, csv_filename);

disp('==================================================');
disp(['SUCCESS! Validation data exported to: ', csv_filename]);
disp('==================================================');