clear all; close all; clc;

%% 1. SYSTEM PARAMETERS (กำหนดตัวแปรเข้าสูตร)
% ให้ Brown เป็น Input (a) และอยู่ติด Ground
L_Ground    = 210; % d
L_Brown     = 118; % a (Input Link - สีน้ำตาล)
L_Blue      = 210; % b (Coupler Link - สีน้ำเงิน)
L_LightBlue = 168; % c (Output Link - สีฟ้าอ่อน)

% Map เข้าตัวแปรสูตร Norton
d = L_Ground;
a = L_Brown;      % กำหนด Brown เป็น a
b = L_Blue;
c = L_LightBlue;

% --- INPUT SPECIFICATION ---
% กำหนดมุมให้กับ Brown Link โดยตรง
theta_input_deg = 81.65; 
theta2 = deg2rad(theta_input_deg); % นี่คือมุมของ Brown Link

%% 2. SOLVER (Standard Forward Kinematics)
% ใช้สูตร A, B, C ตามสไลด์อาจารย์เป๊ะๆ
% หา Theta4 (มุมของ Light Blue) และ Theta3 (มุมของ Blue)

K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- หา Theta 4 (Output - Light Blue) ---
A = cos(theta2) - K1 - K2*cos(theta2) + K3;
B = -2*sin(theta2);
C = K1 - (K2+1)*cos(theta2) + K3;

det = B^2 - 4*A*C;
if det < 0
    error('Assembly failed.');
end

% คำนวณ 2 คำตอบ (Open / Crossed)
t4_1 = 2*atan2(-B + sqrt(det), 2*A);
t4_2 = 2*atan2(-B - sqrt(det), 2*A);

% --- หา Theta 3 (Coupler - Blue) ---
D = cos(theta2) - K1 + K4*cos(theta2) + K5;
E = -2*sin(theta2);
F = K1 + (K4-1)*cos(theta2) + K5;

det3 = E^2 - 4*D*F;
t3_1 = 2*atan2(-E + sqrt(det3), 2*D);
t3_2 = 2*atan2(-E - sqrt(det3), 2*D);

% เลือก Case ที่ต้องการ (ปกติใช้ Case 1 หรือ 2 ลองสลับดูได้ครับ)
% เลือกชุดคำตอบที่ 2 (มักจะเป็น Open สำหรับ configuration นี้)
theta4 = t4_2; 
theta3 = t3_2;

%% 3. PLOTTING
% คำนวณ Vector Position (ใช้ exp ตามสไตล์อาจารย์)
R_Brown     = a * exp(1j * theta2);    % Input (Brown)
R_Blue      = b * exp(1j * theta3);    % Coupler (Blue)
R_LightBlue = c * exp(1j * theta4);    % Output (Light Blue)
R_Ground    = d * exp(1j * 0);         % Ground

% Coordinates
J1 = R_Brown;            % จุดต่อ Brown-Blue
J2 = R_Brown + R_Blue;   % จุดต่อ Blue-LightBlue (คำนวณจากฝั่งซ้าย)
% หรือ J2_check = R_Ground + R_LightBlue; % (คำนวณจากฝั่งขวา)

figure('Color','w','Position',[100 100 800 500]); 
hold on; axis equal; grid on; box on;
title(['Mechanism Analysis: Brown Link as Input (\theta = ' num2str(theta_input_deg) '^\circ)']);
xlabel('X (mm)'); ylabel('Y (mm)');

% --- PLOT VECTORS (Quiver Style) ---

% 1. Ground (สีชมพูเข้ม)
plot([0 real(R_Ground)], [0 imag(R_Ground)], 'Color', [0.8 0 0.5], 'LineWidth', 3, 'LineStyle', '--');
text(0, -20, 'Origin (Pivot 1)'); text(real(R_Ground), -20, 'Pivot 2');

% 2. Brown Link (Input - อยู่ติด Ground ที่จุด Origin)
quiver(0, 0, real(R_Brown), imag(R_Brown), 0, 'Color', [0.6 0.3 0], 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Brown (Input)');

% 3. Blue Link (Coupler)
quiver(real(J1), imag(J1), real(R_Blue), imag(R_Blue), 0, 'b', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Blue');

% 4. Light Blue Link (Output - อยู่ติด Ground อีกฝั่ง)
% วาดจาก Pivot 2 ขึ้นไปหา J2
quiver(real(R_Ground), imag(R_Ground), real(R_LightBlue), imag(R_LightBlue), 0, 'c', 'LineWidth', 5, 'MaxHeadSize', 0.5, 'DisplayName', 'Light Blue');

% Joints
plot(real(J1), imag(J1), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);
plot(real(J2), imag(J2), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 8);

% Display Angle Info
fprintf('========================================\n');
fprintf('RESULTS (Brown as Input)\n');
fprintf('========================================\n');
fprintf('Theta Input (Brown)     : %8.2f deg\n', rad2deg(theta2));
fprintf('Theta Coupler (Blue)    : %8.2f deg\n', rad2deg(theta3));
fprintf('Theta Output (LightBlue): %8.2f deg\n', rad2deg(theta4));
fprintf('========================================\n');

legend('Location','NorthEast');
xlim([-50 350]); ylim([-100 300]);
