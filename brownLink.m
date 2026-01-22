clear all; close all; clc;

%% 1. SYSTEM PARAMETERS
L1 = 210; % d (Ground O2-O4)
L2 = 168; % a (Light Blue Link)
L3 = 210; % b (Blue Link - Coupler)
L4 = 118; % c (Brown Link - INPUT)

% ตัวแปรสำหรับสูตร Norton
d = L1;
a = L4; % Brown is Input
b = L3;
c = L2;

% --- INPUT SPECIFICATION ---
% กำหนดมุมก้านน้ำตาล (ที่ O4) ให้ชี้ลง (ค่าติดลบ)
theta4_deg_at_O4 = -81.65; 
theta_input_rad = deg2rad(theta4_deg_at_O4);

% มุมที่โจทย์ให้มา (Known Angle) วัดที่จุด B เทียบแกน X
% ตามความสัมพันธ์เรขาคณิต (มุมแย้ง/เส้นขนาน):
% มุมที่จุด B (วัดย้อนกลับไป O4) = 180 + มุมที่ O4
known_input_angle_at_B = 180 + theta4_deg_at_O4; 

%% 2. SOLVER (Inverse Kinematics)
% ใช้สูตร Norton หา Theta 3 (Blue) และ Theta 4 (Light Blue) โดยใช้มุมที่ O4 คำนวณ
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Calculate Theta 4 (Light Blue - Output) ---
% หมายเหตุ: ในสูตร Norton มุม Input คือ theta ของก้าน a (Brown @ O4)
A = cos(theta_input_rad) - K1 - K2*cos(theta_input_rad) + K3;
B = -2*sin(theta_input_rad);
C = K1 - (K2+1)*cos(theta_input_rad) + K3;

det = B^2 - 4*A*C;
if det < 0, error('Assembly failed'); end

t4_1 = 2*atan2(-B + sqrt(det), 2*A);
t4_2 = 2*atan2(-B - sqrt(det), 2*A);

% --- Calculate Theta 3 (Blue - Coupler) ---
D = cos(theta_input_rad) - K1 + K4*cos(theta_input_rad) + K5;
E = -2*sin(theta_input_rad);
F = K1 + (K4-1)*cos(theta_input_rad) + K5;

det3 = E^2 - 4*D*F;
t3_1 = 2*atan2(-E + sqrt(det3), 2*D);
t3_2 = 2*atan2(-E - sqrt(det3), 2*D);

% เลือก Case (Case 2: Open Config)
theta_lightblue = t4_2;
theta_blue      = t3_2;

%% 3. PLOTTING
% คำนวณพิกัดเวกเตอร์
R_Ground    = d * exp(1j * 0);
R_Brown     = a * exp(1j * theta_input_rad);    % Input (Brown)
R_Blue      = b * exp(1j * theta_blue);
R_LightBlue = c * exp(1j * theta_lightblue);

% พิกัดจุดต่อ
O4 = R_Ground;          % จุดหมุนของ Brown (210, 0)
J_B = O4 + R_Brown;     % จุด B (ปลายก้านน้ำตาล)
J_A = J_B + R_Blue;     % จุด A (ปลายก้านน้ำเงิน)

figure('Color','w','Position',[100 100 800 600]); 
hold on; axis equal; grid on; box on;
title(['Mechanism Analysis: Input Measured at Point B']);
xlabel('X (mm)'); ylabel('Y (mm)');

% 1. Ground (Pink)
plot([0 real(R_Ground)], [0 imag(R_Ground)], 'Color', [1 0.07 0.57], 'LineWidth', 3, 'LineStyle', '--');
text(0, -15, 'O2'); 
text(real(O4), -15, 'O4');

% 2. Light Blue Link (O2 -> A)
quiver(0, 0, real(R_LightBlue), imag(R_LightBlue), 0, 'c', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Light Blue');

% 3. Blue Link (B -> A) 
quiver(real(J_B), imag(J_B), real(R_Blue), imag(R_Blue), 0, 'b', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Blue');

% 4. Brown Link (O4 -> B)
quiver(real(O4), imag(O4), real(R_Brown), imag(R_Brown), 0, 'Color', [0.6 0.3 0], 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Brown');

% Joints
plot(real(J_A), imag(J_A), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
plot(real(J_B), imag(J_B), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
text(real(J_B)+10, imag(J_B)-10, 'B', 'FontSize', 12, 'FontWeight', 'bold');

% --- HIGHLIGHT: INPUT ANGLE AT POINT B ---
% สร้างเส้นแกน X จำลอง (Dashed Line) ที่จุด B
len_axis = 80;
plot([real(J_B) real(J_B)+len_axis], [imag(J_B) imag(J_B)], 'k--', 'LineWidth', 1);
text(real(J_B)+len_axis, imag(J_B), '+X', 'FontSize', 10);

% วาดส่วนโค้งมุม (Arc) ที่จุด B
% มุมของเวกเตอร์ B->O4 คือ 180 + theta_brown
r_arc = 40;
ang_plot_end = known_input_angle_at_B; 
ang_vec = linspace(0, ang_plot_end, 40);

plot(real(J_B) + r_arc*cos(deg2rad(ang_vec)), imag(J_B) + r_arc*sin(deg2rad(ang_vec)), 'r-', 'LineWidth', 2);

% ข้อความแสดงมุม Input ที่จุด B
text(real(J_B) + 10, imag(J_B) + 25, ...
     ['Input = ' num2str(known_input_angle_at_B, '%.2f') '^\circ'], ...
     'Color', 'r', 'FontWeight', 'bold', 'FontSize', 12);
     
legend('Location','NorthEast');
xlim([-50 350]); ylim([-200 150]);

% แสดงผลลัพธ์ใน Command Window
fprintf('========================================\n');
fprintf('RESULTS\n');
fprintf('========================================\n');
fprintf('Angle at O4 (Theta Brown): %.2f deg\n', theta4_deg_at_O4);
fprintf('Known Input Angle at B   : %.2f deg (180 + Theta Brown)\n', known_input_angle_at_B);
fprintf('Theta Light Blue (Output): %.2f deg\n', rad2deg(theta_lightblue));
fprintf('Theta Blue (Coupler)     : %.2f deg\n', rad2deg(theta_blue));
fprintf('========================================\n');
