clear all;
close all;
clc;

% --- 1. SETUP PARAMETERS (Corrected based on Assignment) ---
% Units in Meters (m) for better scaling
L1 = 0.210; % Ground (210 mm)
L2 = 0.091; % Crank (91 mm)
L3 = 0.236; % Coupler (236 mm)
L4 = 0.180; % Rocker (180 mm)

% Input Conditions (from Assignment Question 1)
q2_deg = 19.94;          % Input angle in degrees
w2 = -2.2;               % Input Velocity (rad/s)
alp2 = -0.8;             % Input Acceleration (rad/s^2)

% Constants for Position Analysis (Norton's Method)
a = L2; b = L3; c = L4; d = L1;
q2 = deg2rad(q2_deg);    % Convert to radians

% --- 2. POSITION ANALYSIS (Your Logic) ---
K1 = d/a;
K2 = d/c;
K3 = (a^2-b^2+c^2+d^2)/(2*a*c);
K4 = d/b;
K5 = (c^2 -d^2 -a^2 -b^2)/(2*a*b);

% Coefficients for Theta 4
A = cos(q2) - K1 - K2*cos(q2) + K3;
B = -2*sin(q2);
C = K1 - (K2+1)*cos(q2) + K3;

% Coefficients for Theta 3
D = cos(q2) - K1 + K4*cos(q2) + K5;
E = -2*sin(q2);
F = K1 + (K4-1)*cos(q2) + K5;

% Calculate Theta 4 (Open & Crossed)
q4_crossed = 2*atan((-B + sqrt(B^2-4*A*C))/(2*A));
q4_open    = 2*atan((-B - sqrt(B^2-4*A*C))/(2*A)); % Check sign for correct config

% Calculate Theta 3 (Open & Crossed)
q3_crossed = 2*atan((-E + sqrt(E^2-4*D*F))/(2*D));
q3_open    = 2*atan((-E - sqrt(E^2-4*D*F))/(2*D)); % Check sign for correct config

% Select OPEN CONFIGURATION for the Assignment (Standard)
q3 = q3_open;
q4 = q4_open;

fprintf('--- Position Results ---\n');
fprintf('Theta 3 (Open): %.4f deg\n', rad2deg(q3));
fprintf('Theta 4 (Open): %.4f deg\n', rad2deg(q4));

% --- 3. VELOCITY ANALYSIS (Added for Q1) ---
% Matrix Form: [J] * [w3; w4] = [V]
J = [-L3*sin(q3),  L4*sin(q4);
      L3*cos(q3), -L4*cos(q4)];
  
V_rhs = [ L2*w2*sin(q2);
         -L2*w2*cos(q2)];

w_sol = J \ V_rhs; % Solve Linear System
w3 = w_sol(1);
w4 = w_sol(2);

fprintf('\n--- Velocity Results ---\n');
fprintf('Omega 3: %.4f rad/s\n', w3);
fprintf('Omega 4: %.4f rad/s\n', w4);

% --- 4. ACCELERATION ANALYSIS (Added for Q1) ---
% Matrix Form: [J] * [alpha3; alpha4] = [Acc_RHS]
A_rhs = [ L2*alp2*sin(q2) + L2*w2^2*cos(q2) + L3*w3^2*cos(q3) - L4*w4^2*cos(q4);
         -L2*alp2*cos(q2) + L2*w2^2*sin(q2) + L3*w3^2*sin(q3) - L4*w4^2*sin(q4)];

alp_sol = J \ A_rhs; % Solve Linear System (Using same J matrix)
alp3 = alp_sol(1);
alp4 = alp_sol(2);

fprintf('\n--- Acceleration Results ---\n');
fprintf('Alpha 3: %.4f rad/s^2\n', alp3);
fprintf('Alpha 4: %.4f rad/s^2\n', alp4);

% --- 5. POINT P CALCULATION (Trajectory Point) ---
P_dist = 0.065; % 65 mm on Link 3
% Assuming P is on the line AB (delta = 0). If offset, add angle here.
Rp = L2*exp(1j*q2) + P_dist*exp(1j*q3); 
P_acc_x = real(-L2*alp2*sin(q2) - L2*w2^2*cos(q2) - P_dist*alp3*sin(q3) - P_dist*w3^2*cos(q3));
P_acc_y = real(L2*alp2*cos(q2) - L2*w2^2*sin(q2) + P_dist*alp3*cos(q3) - P_dist*w3^2*sin(q3));

fprintf('\n--- Point P Results ---\n');
fprintf('P Accel X: %.4f m/s^2\n', P_acc_x);
fprintf('P Accel Y: %.4f m/s^2\n', P_acc_y);

% --- 6. PLOTTING (Based on your style) ---
% Define Vectors for Plotting
RA = L2*exp(1j*q2);
RB = RA + L3*exp(1j*q3);
RO4 = L1*exp(1j*0); % Ground from O2 to O4

% Convert to Cartesian
Ax = real(RA); Ay = imag(RA);
Bx = real(RB); By = imag(RB);
Ox = 0; Oy = 0;
O4x = real(RO4); O4y = imag(RO4);

figure(1);
hold on; grid on; axis equal;
title('Four-Bar Linkage (Open Configuration)');
xlabel('X Position (m)'); ylabel('Y Position (m)');

% Plot Links using plot command (easier to see than quiver for connected links)
plot([Ox, Ax], [Oy, Ay], 'r-o', 'LineWidth', 2, 'DisplayName', 'Crank');     % Link 2
plot([Ax, Bx], [Ay, By], 'b-o', 'LineWidth', 2, 'DisplayName', 'Coupler');   % Link 3
plot([O4x, Bx], [O4y, By], 'k-o', 'LineWidth', 2, 'DisplayName', 'Rocker');  % Link 4
plot([Ox, O4x], [Oy, O4y], 'k--', 'LineWidth', 1.5, 'DisplayName', 'Ground'); % Link 1

% Plot Vector Arrows (Your quiver style - purely for visualization)
quiver(Ox, Oy, Ax, Ay, 0, 'r', 'MaxHeadSize', 0.5);
quiver(Ax, Ay, Bx-Ax, By-Ay, 0, 'b', 'MaxHeadSize', 0.5);
quiver(O4x, O4y, Bx-O4x, By-O4y, 0, 'k', 'MaxHeadSize', 0.5);

legend('Location', 'best');
xlim([-0.1 0.3]); ylim([-0.1 0.2]);
