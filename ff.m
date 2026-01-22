clear all; close all; clc;

%% 1. USER DATA (ข้อมูลตัวเลขของคุณ)
% ใช้หน่วย mm ตามที่คุณให้มา (ไม่แปลงเป็นเมตรเพื่อความสอดคล้อง)
L_Ground = 210;  % d (ระยะ O2-O4)
L_Green = 180;   % a (Link 2 ที่ O2)
L_Yellow = 180;  % b (Link 3 Coupler - เป้าหมายคือมุมนี้ต้องได้ 19.94)
L_Grey = 118;    % c (Link 4 ที่ O4)

% --- TARGET INPUT SPECIFICATION ---
% เป้าหมาย: ต้องการให้มุมของก้านเหลือง (theta3) เป็น 19.94 องศา
target_theta3_deg = 19.94;
target_theta3_rad = deg2rad(target_theta3_deg);

% เงื่อนไขเพิ่มเติม: Green link (theta2) ต้องอยู่ใต้ Ground
% ซึ่งหมายความว่า theta2 ที่เราหาได้ควรเป็นค่าติดลบ (ในช่วง -180 ถึง 0)

%% 2. SOLVER USING NORTON'S METHOD (ตามโค้ดอาจารย์)

% เราจะใช้ fsolve หาค่า theta2 (มุม Input) ที่ทำให้ได้ theta3 ตามเป้าหมาย
% โดยใช้สมการเวกเตอร์ลูป: a*e^(j*t2) + b*e^(j*t3) - c*e^(j*t4) - d = 0

% การเตรียม parameters สำหรับ fsolve
% เราต้องการหา theta2 และ theta4 (2 ตัวแปร) จาก 2 สมการ (Real & Imaginary)
% โดยที่ theta3 เป็นค่าคงที่ที่เรากำหนดไว้แล้ว (target_theta3_rad)

% ฟังก์ชัน Objective สำหรับ fsolve (ต้องเท่ากับ 0)
% x(1) = theta2, x(2) = theta4
objective_func = @(x) [
    L_Green * cos(x(1)) + L_Yellow * cos(target_theta3_rad) - L_Grey * cos(x(2)) - L_Ground; % Real part
    L_Green * sin(x(1)) + L_Yellow * sin(target_theta3_rad) - L_Grey * sin(x(2))             % Imaginary part
];

% การเดาค่าเริ่มต้น (Initial Guess) - สำคัญมากสำหรับการลู่เข้าสู่คำตอบที่ต้องการ
% เราต้องการให้ Green (x(1)) อยู่ใต้กราวด์ เดาเป็นค่าลบ เช่น -60 องศา
% Grey (x(2)) น่าจะอยู่ควอดรันต์ที่ 1 หรือ 2 เดาเป็น 90 องศา
initial_guess = [deg2rad(-60), deg2rad(90)];

% ตั้งค่า options สำหรับ fsolve
options = optimoptions('fsolve', 'Display', 'off', 'FunctionTolerance', 1e-10);

% แก้สมการ
[sol, fval, exitflag] = fsolve(objective_func, initial_guess, options);

if exitflag <= 0
    error('fsolve did not converge to a solution.');
end

% ดึงคำตอบ
theta2_sol_rad = sol(1); % นี่คือมุม Input ของก้านเขียวที่เราหามาได้
theta4_sol_rad = sol(2);

% แปลงเป็นองศาและปรับช่วงให้เหมาะสม
theta2_sol_deg = rad2deg(theta2_sol_rad);
theta4_sol_deg = rad2deg(theta4_sol_rad);

%% 3. VERIFICATION (ตรวจสอบความถูกต้อง)
% ตรวจสอบว่า theta2 ที่ได้ตรงกับเงื่อนไข "below ground" หรือไม่
if theta2_sol_deg > 0 && theta2_sol_deg < 180
    warning('Solution found, but Green link is ABOVE ground. Check initial guess.');
end

%% 4. CALCULATE COORDINATES FOR PLOTTING
O2 = [0; 0];
O4 = [L_Ground; 0];

% ใช้มุมที่แก้ได้มาหาพิกัด
J_Green = O2 + L_Green * [cos(theta2_sol_rad); sin(theta2_sol_rad)];
J_Grey = O4 + L_Grey * [cos(theta4_sol_rad); sin(theta4_sol_rad)];

%% 5. DISPLAY RESULTS
fprintf('========================================\n');
fprintf('RESULTS USING NORTON''S CONCEPT (via fsolve)\n');
fprintf('========================================\n');
fprintf('Target Theta Yellow (Coupler): %8.2f deg\n', target_theta3_deg);
fprintf('Condition: Green below ground\n');
fprintf('----------------------------------------\n');
fprintf('SOLVED ANGLES:\n');
fprintf('Theta Green (at O2):           %8.2f deg\n', theta2_sol_deg);
fprintf('Theta Grey  (at O4):           %8.2f deg\n', theta4_sol_deg);
fprintf('========================================\n');

%% 6. PLOTTING
figure('Color','w','Position',[100 100 700 500]); hold on; axis equal; grid on; box on;
title({'Structure: Green(O2)-Yellow(Coup)-Grey(O4)'; ...
       ['Method: Norton Concept (fsolve) | Target \theta_Y=' num2str(target_theta3_deg) '^\circ']});
xlabel('X (mm)'); ylabel('Y (mm)');

% Ground Line (Pink เข้มตามต้องการ)
plot([-50 O4(1)+50], [0 0], 'Color', [1 0.2 0.6], 'LineWidth', 3, 'LineStyle', '--');

% Pivots
plot(O2(1), O2(2), 'ko', 'MarkerSize', 14, 'MarkerFaceColor','w', 'LineWidth',2);
text(O2(1)-30, O2(2)+20, 'O2', 'FontSize',12,'FontWeight','bold');
plot(O4(1), O4(2), 'ko', 'MarkerSize', 14, 'MarkerFaceColor','w', 'LineWidth',2);
text(O4(1)+10, O4(2)+20, 'O4', 'FontSize',12,'FontWeight','bold');

% Links
plot([O2(1) J_Green(1)], [O2(2) J_Green(2)], 'g-', 'LineWidth', 7, 'DisplayName','Green');
plot([J_Green(1) J_Grey(1)], [J_Green(2) J_Grey(2)], 'y-', 'LineWidth', 7, 'DisplayName',['Yellow (Target \theta=' num2str(target_theta3_deg) '^\circ)']);
plot([O4(1) J_Grey(1)], [O4(2) J_Grey(2)], 'Color', [0.6 0.6 0.6], 'LineWidth', 7, 'DisplayName','Grey');

% Joints
plot(J_Green(1), J_Green(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor','w', 'LineWidth',1.5);
plot(J_Grey(1), J_Grey(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor','w', 'LineWidth',1.5);

% Visualize Input Angle on Yellow Link
plot([J_Green(1) J_Green(1)+60], [J_Green(2) J_Green(2)], 'k--', 'LineWidth', 0.5); 
arc_r = 40; 
% สร้างเส้นโค้งมุม (ต้องระวังทิศทาง)
if target_theta3_rad >= 0
    ang_vec = linspace(0, target_theta3_rad, 20);
else
    ang_vec = linspace(target_theta3_rad, 0, 20);
end
plot(J_Green(1) + arc_r*cos(ang_vec), J_Green(2) + arc_r*sin(ang_vec), 'r-', 'LineWidth', 1.5);
text(J_Green(1) + arc_r+5, J_Green(2) + arc_r*sin(target_theta3_rad/2), ...
     ['\theta_{Y}=' num2str(target_theta3_deg) '^{\circ}'], 'Color', 'r', 'FontSize', 10, 'FontWeight','bold');

legend('Location','NorthEastOutside'); xlim([-50 300]); ylim([-200 150]);
