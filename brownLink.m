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
% จากการคำนวณ Loop 1 ที่ถูกต้อง มุมของก้านที่ O4 จะประมาณ -97.8 องศา
theta_brown_deg = -97.8; 
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

% เลือก Case (ลองเลือก Case 2)
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
O4 = R_Ground;         % จุดหมุนของ Brown
J_B = O4 + R_Brown;    % จุด B (ปลายก้านน้ำตาล)
J_A = J_B + R_Blue;    % จุด A (ปลายก้านน้ำเงิน) - ควรตรงกับ R_LightBlue จาก O2

figure('Color','w','Position',[100 100 800 600]); 
hold on; axis equal; grid on; box on;
title(['Loop 2 Analysis: Brown Input \theta = ' num2str(theta_brown_deg) '^\circ']);
xlabel('X (mm)'); ylabel('Y (mm)');

% 1. Ground (Pink)
plot([0 real(R_Ground)], [0 imag(R_Ground)], 'Color', [1 0.07 0.57], 'LineWidth', 3, 'LineStyle', '--');
text(0, 10, 'O2'); 
text(real(O4), 10, 'O4');

% 2. Light Blue Link (O2 -> A)
quiver(0, 0, real(R_LightBlue), imag(R_LightBlue), 0, 'c', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Light Blue');

% 3. Blue Link (A -> B)
% หมายเหตุ: วาดจาก B ไป A หรือ A ไป B ก็ได้ (ในที่นี้วาดตาม Vector Loop O4->B->A)
quiver(real(J_B), imag(J_B), real(R_Blue), imag(R_Blue), 0, 'b', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Blue');

% 4. Brown Link (O4 -> B)
quiver(real(O4), imag(O4), real(R_Brown), imag(R_Brown), 0, 'Color', [0.6 0.3 0], 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Brown');

% Joints
plot(real(J_A), imag(J_A), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
plot(real(J_B), imag(J_B), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
text(real(J_B)+10, imag(J_B), 'B', 'FontSize', 12, 'FontWeight', 'bold');

% --- HIGHLIGHT: ANGLE X MEASURED AT POINT B ---
% วาดเส้นแกน X จำลอง (Dashed Line) ที่จุด B
len_axis = 60;
plot([real(J_B) real(J_B)+len_axis], [imag(J_B) imag(J_B)], 'k--', 'LineWidth', 1);
text(real(J_B)+len_axis, imag(J_B), 'x''', 'FontSize', 10);

% วาดส่วนโค้งมุม (Arc) ที่จุด B
% มุมของก้าน Brown คือ theta2 (วัดจากแกน X)
% เนื่องจากเส้นขนาน มุมแย้งเท่ากัน มุมที่ B เทียบกับแกน X' ก็คือ theta2 เช่นกัน
r_arc = 40;
ang_start = 180; % มุมเริ่มของเส้นประ (ซ้าย) หรือ 0 (ขวา) -> ใช้ 0
ang_vec = linspace(0, theta_brown_deg + 180, 30); % ปรับการแสดงผลให้ดูง่าย (มุมแย้ง)
% หรือวาดมุมของก้านเทียบกับแกน X ที่จุด B (ซึ่งคือมุม theta2 + 180 ถ้ามองย้อนกลับ หรือ theta2 ถ้ามองทิศเดิม)
% เพื่อความชัดเจน วาดมุม theta2 ที่ขนานกับ O4
ang_vec_plot = linspace(0, theta2, 30);
% แต่จุดศูนย์กลางอยู่ที่ B?
% ถ้าจะวัดที่ B ต้องวัดมุมของก้านเทียบกับแกนราบ
% ก้าน Brown เอียง theta2
plot(real(J_B) - r_arc*cos(ang_vec_plot), imag(J_B) - r_arc*sin(ang_vec_plot), 'r-', 'LineWidth', 2);
text(real(J_B) - r_arc, imag(J_B) - 15, ['Angle x (' num2str(theta_brown_deg) '^\circ)'], 'Color', 'r', 'FontWeight', 'bold');

legend('Location','NorthEast');
xlim([-50 300]); ylim([-200 150]);
