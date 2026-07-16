% set_common_params.m
% Single source of truth for the Double Pendulum validation.
% These values strictly match the C++ TwoLinkArm constructor and main.cpp.

% clear; clc;

%% 1. Link Masses (kg)
m1 = 1.0; 
m2 = 1.0; 

%% 2. Link Geometries (m)
l1 = 1.0;  % Total length of Link 1
l2 = 1.0;  % Total length of Link 2

% Center of Mass distances from the respective joint
lc1 = 0.5; 
lc2 = 0.5; 

%% 3. Moments of Inertia (kg*m^2)
% Assumed about the center of mass
I1 = 0.1; 
I2 = 0.1; 

%% 4. Environment Parameters
g = 9.81;  % Gravity (m/s^2)

%% 5. Initial Conditions (Matching main.cpp)
% Both links start completely horizontal, dropped from rest
theta1_init = 0.0; % rad
theta2_init = 0.0; % rad
theta1_dot_init = 0.0; % rad/s
theta2_dot_init = 0.0; % rad/s

%% 6. Solver / Simulation Parameters
dt = 0.001;      % 1 ms timestep matching C++ RK4
end_time = 5.0;  % 5 seconds of total simulation time

disp('Validation parameters successfully loaded into the Workspace.');