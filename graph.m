clear all;
close all;
clc;

%% --- 1. SETUP PARAMETERS (Corrected for Assignment Q1) ---
% Units: Meters (m)
L1 = 0.210; % Ground
L2 = 0.091; % Crank
L3 = 0.236; % Coupler
L4 = 0.180; % Rocker

% Norton's Notation
a = L2; b = L3; c = L4; d = L1;

% Input Conditions (Question 1)
q2_deg = 19.94;
w2     = -2.2;      % rad/s
alpha2 = -0.8;      % rad/s^2

q2 = deg2rad(q2_deg); % Convert to radians

%% --- 2. POSITION ANALYSIS (Vector Loop) ---
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

% Calculate Angles (OPEN Configuration)
% Note: Using signs for Open Config based on standard form
q4 = 2*atan((-B - sqrt(B^2-4*A*C))/(2*A)); 
q3 = 2*atan((-E - sqrt(E^2-4*D*F))/(2*D));

fprintf('--- Position Results ---\n');
fprintf('Theta 3: %.4f deg\n', rad2deg(q3));
fprintf('Theta 4: %.4f deg\n', rad2deg(q4));

%% --- 3. VELOCITY ANALYSIS (Matrix Method) ---
% J * w = V
J = [-b*sin(q3),  c*sin(q4);
      b*cos(q3), -c*cos(q4)];
  
V_RHS = [ a*w2*sin(q2);
         -a*w2*cos(q2)];

w_sol = J \ V_RHS; 
w3 = w_sol(1);
w4 = w_sol(2);

fprintf('\n--- Velocity Results ---\n');
fprintf('Omega 3: %.4f rad/s\n', w3);
fprintf('Omega 4: %.4f rad/s\n', w4);

%% --- 4. ACCELERATION ANALYSIS (Matrix Method) ---
% J * alpha = A_RHS
A_RHS = [ a*alpha2*sin(q2) + a*w2^2*cos(q2) + b*w3^2*cos(q3) - c*w4^2*cos(q4);
         -a*alpha2*cos(q2) + a*w2^2*sin(q2) + b*w3^2*sin(q3) - c*w4^2*sin(q4)];

alpha_sol = J \ A_RHS;
alpha3 = alpha_sol(1);
alpha4 = alpha_sol(2);

fprintf('\n--- Acceleration Results ---\n');
fprintf('Alpha 3: %.4f rad/s^2\n', alpha3);
fprintf('Alpha 4: %.4f rad/s^2\n', alpha4);

%% --- 5. VECTOR CALCULATION (For Plotting) ---
% 5.1 Position Vectors
RA  = a*exp(1j*q2);
RBA = b*exp(1j*q3);
RB  = RA + RBA; % Position of Joint B
RO4 = d*exp(1j*0);

% 5.2 Velocity Vectors (V = w x r)
VA  = 1j * a * w2 * exp(1j*q2);
VBA = 1j * b * w3 * exp(1j*q3);
VB  = VA + VBA; % Absolute Velocity of B

% 5.3 Acceleration Vectors
AA  = (1j*a*alpha2 - a*w2^2) * exp(1j*q2);
ABA = (1j*b*alpha3 - b*w3^2) * exp(1j*q3);
AB  = AA + ABA; % Absolute Acceleration of B

% Extract Components (Real/Imag)
RAx = real(RA); RAy = imag(RA);
RBx = real(RB); RBy = imag(RB);
O4x = real(RO4); O4y = imag(RO4);

VAx = real(VA); VAy = imag(VA);
VBx = real(VB); VBy = imag(VB);

AAx = real(AA); AAy = imag(AA);
ABx = real(AB); ABy = imag(AB);

%% --- 6. COMBINED PLOT ---
figure(1); clf;
hold on; grid on; axis equal;
title(['Four-Bar Mechanism Analysis (Combined Plot)', newline, ...
       'Links (Blue/Red), Velocity (Green), Acceleration (Orange)']);
xlabel('X (m)'); ylabel('Y (m)');

% 6.1 Plot LINKS (Mechanism Geometry)
plot([0, RAx],   [0, RAy],   'r-o', 'LineWidth', 3, 'MarkerSize', 8); % Crank
plot([RAx, RBx], [RAy, RBy], 'b-o', 'LineWidth', 3, 'MarkerSize', 8); % Coupler
plot([O4x, RBx], [O4y, RBy], 'k-o', 'LineWidth', 3, 'MarkerSize', 8); % Rocker
plot([0, O4x],   [0, O4y],   'k--', 'LineWidth', 1.5);                % Ground

% 6.2 Plot VELOCITY Vectors (Green)
% Scale Factor: ปรับตัวเลขนี้ถ้าลูกศรยาวหรือสั้นไป
S_vel = 0.1; 
quiver(RAx, RAy, VAx*S_vel, VAy*S_vel, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Velocity');
quiver(RBx, RBy, VBx*S_vel, VBy*S_vel, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% 6.3 Plot ACCELERATION Vectors (Orange/Magenta)
% Scale Factor: ปรับตัวเลขนี้ถ้าลูกศรยาวหรือสั้นไป
S_acc = 0.05; 
quiver(RAx, RAy, AAx*S_acc, AAy*S_acc, 0, 'm', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Acceleration');
quiver(RBx, RBy, ABx*S_acc, ABy*S_acc, 0, 'm', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Add Legend & Styling
legend('Crank (L2)', 'Coupler (L3)', 'Rocker (L4)', 'Ground', 'Velocity Vectors', 'Acceleration Vectors', 'Location', 'best');
axis([-0.1 0.3 -0.15 0.25]); % Set view frame

% Text Labels
text(0, 0, ' O2', 'VerticalAlignment','top');
text(O4x, O4y, ' O4', 'VerticalAlignment','top');
text(RAx, RAy, ' A');
text(RBx, RBy, ' B');
