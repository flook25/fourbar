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
D = cos(theta_input_rad
