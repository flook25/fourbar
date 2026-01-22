clear all; close all; clc;

%% 1. SYSTEM PARAMETERS
L1 = 210; % d (Ground O2-O4)
L2 = 168; % a (Light Blue Link)
L3 = 210; % b (Blue Link - Coupler)
L4 = 118; % c (Brown Link - INPUT)

% กำหนดตัวแปรสำหรับเข้าสูตร
d = L1;
a = L2;
b = L3;
c = L4;

% --- INPUT SPECIFICATION (AT BROWN LINK) ---
% เงื่อนไข: ก้านน้ำตาลอยู่ที่ O4 (ปลายลูกศร Ground) และชี้ลง (มุมติดลบ)
theta4_deg = -81.65;      
theta4 = deg2rad(theta4_deg); 

%% 2. SOLVER (Inverse Kinematics)
% เราต้องการหา Theta2 (Light Blue) โดยรู้ Theta4 (Brown) ที่ O4

% คำนวณสัมประสิทธิ์ A, B, C (Inverse Case)
% สมการ: R_LightBlue + R_Blue = R_Ground + R_Brown
A_inv = -2 * a * c * sin(theta4);
B_inv = -2 * a * (c * cos(theta4) + d);
C_inv = a^2 + c^2 + d^2 - b^2 + 2 * c * d * cos(theta4);

% ใช้สูตร Quadratic (t = tan(theta2/2))
A_quad = C_inv - B_inv;
B_quad = 2 * A_inv;
C_quad = C_inv + B_inv;

det_quad = B_quad^2 - 4*A_quad*C_quad;

if det_quad < 0
    error('Assembly failed: Cannot assemble mechanism with this Brown Link angle.');
end

% --- คำนวณหา Theta2 (Light Blue) ---
% หาค่า t = tan(theta2 / 2)
t_sol1 = (-B_quad + sqrt(det_quad)) / (2*A_quad);
t_sol2 = (-B_quad - sqrt(det_quad)) / (2*A_quad);

% แปลงกลับเป็นมุม theta2
theta2_sol1 = 2 * atan(t_sol1);
theta2_sol2 = 2 * atan(t_sol2);

% --- คำนวณหา Theta3 (Blue - Coupler) ---
% จาก Vector Loop: R_Blue = R_Ground + R_Brown - R_LightBlue
get_theta3 = @(th2) angle(d + c*exp(1j*theta4) - a*exp(1j*th2));

theta3_sol1 = get_theta3(theta2_sol1);
theta3_sol2 = get_theta3(theta2_sol2);

%% 3. PLOTTING
figure('Color','w','Position',[100 100 1000 500]); 

% --- PLOT CASE 1 ---
subplot(1,2,1); hold on; axis equal; grid on;
title(['Case 1: Brown Input at O4 (\theta = ' num2str(theta4_deg) '^\circ)']);
plot_mechanism(a, b, c, d, theta2_sol1, theta3_sol1, theta4);
xlim([-100 350]); ylim([-250 150]); 

% --- PLOT CASE 2 ---
subplot(1,2,2); hold on; axis equal; grid on;
title(['Case 2: Brown Input at O4 (\theta = ' num2str(theta4_deg) '^\circ)']);
plot_mechanism(a, b, c, d, theta2_sol2, theta3_sol2, theta4);
xlim([-100 350]); ylim([-250 150]);

% แสดงผลลัพธ์
fprintf('========================================\n');
fprintf('RESULTS (Brown Link Input at O4)\n');
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
    R_LightBlue = a * exp(1j * th2);    % Light Blue (a)
    R_Blue      = b * exp(1j * th3);    % Blue (b)
    R_Brown     = c * exp(1j * th4);    % Brown (c)
    R_Ground    = d * exp(1j * 0);      % Ground (d)

    % Coordinates
    J1 = R_LightBlue;              % Joint between Light Blue & Blue
    J2 = R_Ground + R_Brown;       % Joint between Brown & Blue (Calculated from O4)
    
    % 1. Plot Ground (O2 -> O4)
    % วาดเส้น Ground สีชมพูเข้ม
    plot([0 real(R_Ground)], [0 imag(R_Ground)], 'Color', [0.8 0 0.5], 'LineWidth', 3, 'LineStyle', '--');
    text(0, -20, 'O2'); 
    text(real(R_Ground), -20, 'O4 (Tip of Arrow)');

    % 2. วาดเส้นอ้างอิงแกน X ที่ O4 (เพื่อให้เห็นมุมชัดเจน)
    plot([real(R_Ground) real(R_Ground)+100], [0 0], 'k:', 'LineWidth', 1);

    % 3. Link 2 (Light Blue) - จาก O2
    quiver(0, 0, real(R_LightBlue), imag(R_LightBlue), 0, 'c', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Light Blue');
    
    % 4. Link 3 (Blue) - จาก J1 ไป J2
    quiver(real(J1), imag(J1), real(R_Blue), imag(R_Blue), 0, 'b', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Blue');
    
    % 5. Link 4 (Brown) - จาก O4 (Input)
    % จุดเริ่มต้นคือ O4 (real(R_Ground), imag(R_Ground))
    quiver(real(R_Ground), imag(R_Ground), real(R_Brown), imag(R_Brown), 0, 'Color', [0.6 0.3 0], 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Brown (Input)');

    % Joints
    plot(real(J1), imag(J1), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
    plot(real(J2), imag(J2), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
    
    % 6. Draw Input Angle Arc at O4 (Relative to X-axis)
    r_arc = 50;
    ang_vec = linspace(0, th4, 30);
    % วาดส่วนโค้งมุมจากแกน X ที่ O4
    plot(real(R_Ground) + r_arc*cos(ang_vec), imag(R_Ground) + r_arc*sin(ang_vec), 'r-', 'LineWidth', 2);
    % ใส่ข้อความบอกมุม
    text(real(R_Ground) + r_arc + 5, imag(R_Ground) + r_arc*sin(th4/2) - 10, ...
         ['\theta_{in}=' num2str(rad2deg(th4)) '^\circ'], 'Color', 'r', 'FontWeight', 'bold');
end
