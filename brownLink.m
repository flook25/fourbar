clear all; close all; clc;

%% 1. SYSTEM PARAMETERS
L1 = 210; % d (Ground)
L2 = 168; % a (Light Blue Link)
L3 = 210; % b (Blue Link - Coupler)
L4 = 118; % c (Brown Link - NOW INPUT)

% กำหนดตัวแปรสำหรับเข้าสูตร
d = L1;
a = L2;
b = L3;
c = L4;

% --- INPUT SPECIFICATION (AT BROWN LINK) ---
theta4_deg = 81.65;      % กำหนดมุม Input ที่ก้าน Brown
theta4 = deg2rad(theta4_deg); 

%% 2. SOLVER (Inverse Kinematics: Known Theta4 -> Find Theta2, Theta3)
% เราต้องการหา Theta2 (Light Blue) โดยรู้ Theta4 (Brown)
% จัดรูปสมการ Vector Loop ใหม่เพื่อหา Theta2:
% รูปแบบสมการ: A*sin(t2) + B*cos(t2) + C = 0

% คำนวณสัมประสิทธิ์ A, B, C สำหรับ Inverse Case
% (Derived from: |R_Input + R_Coupler|^2 = |R_Output + R_Ground|^2 rearranged)
A_inv = -2 * a * c * sin(theta4);
B_inv = -2 * a * (c * cos(theta4) + d);
C_inv = a^2 + c^2 + d^2 - b^2 + 2 * c * d * cos(theta4);

% ตรวจสอบ Discriminant
det = B_inv^2 + A_inv^2 - C_inv^2; % Note: สูตรสำหรับรูปแบบ A sin + B cos + C = 0 จะต่างจาก Quadratic ปกตินิดหน่อย
% หรือใช้สูตร Half-tangent identity (t = tan(theta/2)) จะได้รูปแบบ Quadratic:
% (C - B)t^2 + (2A)t + (C + B) = 0

% ใช้สูตร Quadratic จากการแทนค่า t = tan(theta2/2)
A_quad = C_inv - B_inv;
B_quad = 2 * A_inv;
C_quad = C_inv + B_inv;

det_quad = B_quad^2 - 4*A_quad*C_quad;

if det_quad < 0
    error('Assembly failed: Cannot assemble mechanism with this Brown Link angle.');
end

% --- คำนวณหา Theta2 (Light Blue) ---
% Case 1
t_sol1 = (-B_quad + sqrt(det_quad)) / (2*A_quad);
theta2_sol1 = 2 * atan(t_sol1);

% Case 2
t_sol2 = (-B_quad - sqrt(det_quad)) / (2*A_quad);
theta2_sol2 = 2 * atan(t_sol2);

% --- คำนวณหา Theta3 (Blue - Coupler) ---
% ใช้ Vector Loop: R_Blue = R_Brown + R_Ground - R_LightBlue
% b*e^j*t3 = c*e^j*t4 + d - a*e^j*t2
get_theta3 = @(th2) angle(c*exp(1j*theta4) + d - a*exp(1j*th2));

theta3_sol1 = get_theta3(theta2_sol1);
theta3_sol2 = get_theta3(theta2_sol2);

%% 3. PLOTTING
figure('Color','w','Position',[100 100 1000 500]); 

% --- PLOT CASE 1 ---
subplot(1,2,1); hold on; axis equal; grid on;
title(['Case 1: Input \theta_4 (Brown) = ' num2str(theta4_deg) '^\circ']);
plot_mechanism(a, b, c, d, theta2_sol1, theta3_sol1, theta4);
xlim([-100 350]); ylim([-150 250]);

% --- PLOT CASE 2 ---
subplot(1,2,2); hold on; axis equal; grid on;
title(['Case 2: Input \theta_4 (Brown) = ' num2str(theta4_deg) '^\circ']);
plot_mechanism(a, b, c, d, theta2_sol2, theta3_sol2, theta4);
xlim([-100 350]); ylim([-150 250]);

% แสดงผลลัพธ์
fprintf('========================================\n');
fprintf('INVERSE KINEMATICS RESULTS (Input at Brown)\n');
fprintf('Input Theta 4 (Brown): %.2f deg\n', theta4_deg);
fprintf('========================================\n');
fprintf('CASE 1:\n');
fprintf('  Theta 2 (Light Blue): %8.2f deg\n', rad2deg(theta2_sol1));
fprintf('  Theta 3 (Blue)      : %8.2f deg\n', rad2deg(theta3_sol1));
fprintf('----------------------------------------\n');
fprintf('CASE 2:\n');
fprintf('  Theta 2 (Light Blue): %8.2f deg\n', rad2deg(theta2_sol2));
fprintf('  Theta 3 (Blue)      : %8.2f deg\n', rad2deg(theta3_sol2));
fprintf('========================================\n');

%% --- INTERNAL FUNCTION FOR PLOTTING ---
function plot_mechanism(a, b, c, d, th2, th3, th4)
    % Position Vectors
    R_Input     = a * exp(1j * th2);    % Light Blue (a)
    R_Coupler   = b * exp(1j * th3);    % Blue (b)
    R_Output    = c * exp(1j * th4);    % Brown (c)
    R_Ground    = d * exp(1j * 0);      % Ground (d)

    % Coordinates
    J1 = R_Input;              % Joint between Light Blue & Blue
    J2 = R_Ground + R_Output;  % Joint between Brown & Blue (Calculated from O4)
    
    % Plot Ground (Dark Pink as requested previously)
    plot([0 real(R_Ground)], [0 imag(R_Ground)], 'Color', [0.8 0 0.5], 'LineWidth', 3, 'LineStyle', '--');
    text(0, -20, 'O2'); text(real(R_Ground), -20, 'O4');

    % Link 2 (Light Blue)
    quiver(0, 0, real(R_Input), imag(R_Input), 0, 'c', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Light Blue');
    
    % Link 3 (Blue)
    quiver(real(J1), imag(J1), real(R_Coupler), imag(R_Coupler), 0, 'b', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Blue');
    
    % Link 4 (Brown) - Plot from O4
    quiver(real(R_Ground), imag(R_Ground), real(R_Output), imag(R_Output), 0, 'Color', [0.6 0.3 0], 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Brown (Input)');

    % Joints
    plot(real(J1), imag(J1), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
    plot(real(J2), imag(J2), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
    
    % Draw Input Angle Arc at Brown Link
    r_arc = 40;
    plot(real(R_Ground) + r_arc*cos(linspace(0,th4,20)), imag(R_Ground) + r_arc*sin(linspace(0,th4,20)), 'r-');
end
