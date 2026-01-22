clear all; close all; clc;

%% 1. USER DATA (ข้อมูลตัวเลขของคุณ)
L_Ground = 210;  % Distance O2-O4
L_Green = 180;   % Link at O2
L_Yellow = 180;  % Coupler Link
L_Grey = 118;    % Link at O4

% --- INPUT SPECIFICATION ---
% มุม Input ถูกกำหนดที่ "Yellow Link" (Coupler)
theta_Yellow_deg = 19.94;
theta_Yellow = deg2rad(theta_Yellow_deg);

%% 2. GEOMETRIC SOLVER (หาพิกัดจากมุมก้านเหลือง)
O2 = [0; 0];
O4 = [L_Ground; 0];

% กำหนดเวกเตอร์ของก้านเหลือง
V_Yellow = L_Yellow * [cos(theta_Yellow); sin(theta_Yellow)];

% จุดศูนย์กลางเสมือนสำหรับวงกลมที่ 2
Center2 = O4 - V_Yellow;

% พารามิเตอร์สำหรับหาจุดตัดวงกลม
R1 = L_Green; R2 = L_Grey;
Cx = Center2(1); Cy = Center2(2);
D_centers = norm(Center2 - O2);

if D_centers > (R1 + R2) || D_centers < abs(R1 - R2) || D_centers == 0
    error('Assembly failed: Cannot connect links with given Yellow angle.');
end

% คำนวณจุดตัด
a_intersect = (R1^2 - R2^2 + D_centers^2) / (2 * D_centers);
h_intersect = sqrt(max(0, R1^2 - a_intersect^2));
x0 = O2(1) + a_intersect * (Cx - O2(1)) / D_centers;
y0 = O2(2) + a_intersect * (Cy - O2(2)) / D_centers;

J_Green_Sol1 = [x0 + h_intersect * (Cy - O2(2)) / D_centers;
                y0 - h_intersect * (Cx - O2(1)) / D_centers];
J_Green_Sol2 = [x0 - h_intersect * (Cy - O2(2)) / D_centers;
                y0 + h_intersect * (Cx - O2(1)) / D_centers];

% --- SELECTION: Choose solution where Green is BELOW ground ---
if J_Green_Sol1(2) < J_Green_Sol2(2)
    J_Green = J_Green_Sol1;
else
    J_Green = J_Green_Sol2;
end

% คำนวณจุดปลายก้านเทา
J_Grey = J_Green + V_Yellow;

%% 3. CALCULATE REMAINING ANGLES (คำนวณมุมที่เหลือจากพิกัด)
% ใช้ atan2(dy, dx) เพื่อหามุมเทียบกับแกน X

% มุมของก้านเขียว (วัดจาก O2 ไป J_Green)
vec_Green = J_Green - O2;
theta_Green_rad = atan2(vec_Green(2), vec_Green(1));
theta_Green_deg = rad2deg(theta_Green_rad);

% มุมของก้านเทา (วัดจาก O4 ไป J_Grey)
vec_Grey = J_Grey - O4;
theta_Grey_rad = atan2(vec_Grey(2), vec_Grey(1));
theta_Grey_deg = rad2deg(theta_Grey_rad);

%% 4. DISPLAY RESULTS (แสดงผลลัพธ์มุมทั้งหมด)
fprintf('========================================\n');
fprintf('RESULTS FOR LOOP 1 CONFIGURATION\n');
fprintf('========================================\n');
fprintf('Condition: Green below ground\n');
fprintf('----------------------------------------\n');
fprintf('INPUT ANGLE (Yellow Coupler): %8.2f deg\n', theta_Yellow_deg);
fprintf('----------------------------------------\n');
fprintf('CALCULATED ANGLES:\n');
fprintf('Theta Green (at O2):          %8.2f deg\n', theta_Green_deg);
fprintf('Theta Grey  (at O4):          %8.2f deg\n', theta_Grey_deg);
fprintf('========================================\n');

%% 5. PLOTTING
figure('Color','w','Position',[100 100 700 500]); hold on; axis equal; grid on; box on;
title({'Structure: Green(O2)-Yellow(Coup)-Grey(O4)'; ...
       ['Input \theta_Y=' num2str(theta_Yellow_deg) '^\circ | Result: \theta_G=' num2str(theta_Green_deg,'%.1f') '^\circ, \theta_{Gy}=' num2str(theta_Grey_deg,'%.1f') '^\circ']});
xlabel('X (mm)'); ylabel('Y (mm)');

% Ground & Pivots
plot([-50 O4(1)+50], [0 0], 'k:', 'LineWidth', 1);
plot(O2(1), O2(2), 'ko', 'MarkerSize', 14, 'MarkerFaceColor','w', 'LineWidth',2);
text(O2(1)-30, O2(2)+20, 'O2', 'FontSize',12,'FontWeight','bold');
plot(O4(1), O4(2), 'ko', 'MarkerSize', 14, 'MarkerFaceColor','w', 'LineWidth',2);
text(O4(1)+10, O4(2)+20, 'O4', 'FontSize',12,'FontWeight','bold');

% Links
plot([O2(1) J_Green(1)], [O2(2) J_Green(2)], 'g-', 'LineWidth', 7, 'DisplayName','Green');
plot([J_Green(1) J_Grey(1)], [J_Green(2) J_Grey(2)], 'y-', 'LineWidth', 7, 'DisplayName','Yellow (Input)');
plot([O4(1) J_Grey(1)], [O4(2) J_Grey(2)], 'Color', [0.6 0.6 0.6], 'LineWidth', 7, 'DisplayName','Grey');

% Joints
plot(J_Green(1), J_Green(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor','w', 'LineWidth',1.5);
plot(J_Grey(1), J_Grey(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor','w', 'LineWidth',1.5);

% Visualize Input Angle
plot([J_Green(1) J_Green(1)+60], [J_Green(2) J_Green(2)], 'k--', 'LineWidth', 0.5); 
arc_r = 40; ang_vec = linspace(0, theta_Yellow, 20);
plot(J_Green(1) + arc_r*cos(ang_vec), J_Green(2) + arc_r*sin(ang_vec), 'r-', 'LineWidth', 1.5);
text(J_Green(1) + arc_r+5, J_Green(2) + arc_r*sin(theta_Yellow/2), ...
     ['\theta_{Y}=' num2str(theta_Yellow_deg) '^{\circ}'], 'Color', 'r', 'FontSize', 10, 'FontWeight','bold');

legend('Location','NorthEastOutside'); xlim([-50 300]); ylim([-200 150]);
