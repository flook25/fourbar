clear all; close all; clc;

%% 1. SYSTEM PARAMETERS (Loop 2)
L1 = 210; % Ground (d)
L2 = 168; % Light Blue (c) -> Output
L3 = 210; % Blue (b) -> Coupler
L4 = 118; % Brown (a) -> Input

% Map Variables for Norton's Formula
d = L1;
a = L4; % Brown is Input
b = L3;
c = L2;

% --- INPUT SPECIFICATION ---
% "Link brown runs down below ground"
% มุมของก้านที่ O4 คือ -81.65 องศา
theta_brown_deg = -81.65; 
theta2 = deg2rad(theta_brown_deg); 

%% 2. SOLVER (Brown Input)
% ใช้สูตร Norton หา Theta 3 (Blue) และ Theta 4 (Light Blue)
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Calculate Theta 4 (Light Blue - Output) ---
A = cos(theta2) - K1 - K2*cos(theta2) + K3;
B = -2*sin(theta2);
C = K1 - (K2+1)*cos(theta2) + K3;

det = B^2 - 4*A*C;
if det < 0, error('Assembly failed'); end

t4_1 = 2*atan2(-B + sqrt(det), 2*A);
t4_2 = 2*atan2(-B - sqrt(det), 2*A);

% --- Calculate Theta 3 (Blue - Coupler) ---
D = cos(theta2) - K1 + K4*cos(theta2) + K5;
E = -2*sin(theta2);
F = K1 + (K4-1)*cos(theta2) + K5;

det3 = E^2 - 4*D*F;
t3_1 = 2*atan2(-E + sqrt(det3), 2*D);
t3_2 = 2*atan2(-E - sqrt(det3), 2*D);

% เลือก Case (ลองเลือก Case 2 ซึ่งมักจะเป็น Open configuration สำหรับค่านี้)
theta_lightblue = t4_2;
theta_blue      = t3_2;

%% 3. PLOTTING
% คำนวณพิกัดเวกเตอร์
R_Ground    = d * exp(1j * 0);
R_Brown     = a * exp(1j * theta2);    % Input (Brown)
R_Blue      = b * exp(1j * theta_blue);
R_LightBlue = c * exp(1j * theta_lightblue);

% พิกัดจุดต่อ
% O2 = (0,0)
O4 = R_Ground;         % จุดหมุนของ Brown (210, 0)
J_B = O4 + R_Brown;    % จุด B (ปลายก้านน้ำตาล)
J_A = J_B + R_Blue;    % จุด A (ปลายก้านน้ำเงิน) - ควรตรงกับ R_LightBlue จาก O2

figure('Color','w','Position',[100 100 800 600]); 
hold on; axis equal; grid on; box on;
title(['Loop 2 Analysis: Brown Input \theta = ' num2str(theta_brown_deg) '^\circ']);
xlabel('X (mm)'); ylabel('Y (mm)');

% 1. Ground (Pink) - O2 to O4
plot([0 real(R_Ground)], [0 imag(R_Ground)], 'Color', [1 0.07 0.57], 'LineWidth', 3, 'LineStyle', '--');
text(0, 10, 'O2'); 
text(real(O4), 10, 'O4');

% 2. Light Blue Link (O2 -> A)
quiver(0, 0, real(R_LightBlue), imag(R_LightBlue), 0, 'c', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Light Blue');

% 3. Blue Link (A -> B) หรือ (B -> A)
% วาดจาก B ไปหา A (เพื่อให้ลูกศรต่อกัน O4->B->A)
quiver(real(J_B), imag(J_B), real(R_Blue), imag(R_Blue), 0, 'b', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Blue');

% 4. Brown Link (O4 -> B)
quiver(real(O4), imag(O4), real(R_Brown), imag(R_Brown), 0, 'Color', [0.6 0.3 0], 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Brown (Input)');

% Joints
plot(real(J_A), imag(J_A), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
plot(real(J_B), imag(J_B), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
text(real(J_B)+10, imag(J_B)-10, 'B', 'FontSize', 12, 'FontWeight', 'bold');

% --- HIGHLIGHT: MEASURE ANGLE X AT POINT B ---
% สร้างเส้นแกน X จำลอง (Dashed Line) ที่จุด B
len_axis = 80;
% วาดเส้นประแนวนอนที่จุด B
plot([real(J_B) real(J_B)+len_axis], [imag(J_B) imag(J_B)], 'k--', 'LineWidth', 1);
text(real(J_B)+len_axis, imag(J_B), 'x''', 'FontSize', 10);

% วาดส่วนโค้งมุม (Arc) ที่จุด B
% มุมแย้ง: เส้นขนานกัน มุมที่ B เทียบกับแกนราบ ก็มีขนาดเท่ากับมุมที่ O4
% วาดส่วนโค้งจากแกนราบที่ B ไปหาก้าน Brown
r_arc = 40;
% สร้างเวกเตอร์มุมสำหรับวาดส่วนโค้ง (ย้อนกลับไปหาก้าน)
% ก้านเอียง -81.65 องศา (ลงล่างขวา)
% มุมที่จุด B เทียบกับแกน +x' จะเป็นมุมแย้ง 
ang_vec_plot = linspace(180, 180 + theta_brown_deg, 30); 

% วาดส่วนโค้ง
plot(real(J_B) + r_arc*cos(deg2rad(ang_vec_plot)), imag(J_B) + r_arc*sin(deg2rad(ang_vec_plot)), 'r-', 'LineWidth', 2);

% ใส่ข้อความบอกมุม
text(real(J_B) - 20, imag(J_B) - 30, ['\theta = ' num2str(theta_brown_deg) '^\circ'], 'Color', 'r', 'FontWeight', 'bold');

legend('Location','NorthEast');
xlim([-50 350]); ylim([-200 150]);
