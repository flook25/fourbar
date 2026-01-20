clear all
close all
clc

% --- 1. SETUP PARAMETERS (Corrected for Assignment) ---
% Link Lengths in Meters (m)
L1 = 0.210; [cite_start]% Ground (210 mm) [cite: 20]
L2 = 0.091; [cite_start]% Crank (91 mm)   [cite: 20]
L3 = 0.236; [cite_start]% Coupler (236 mm)[cite: 20]
L4 = 0.180; [cite_start]% Rocker (180 mm) [cite: 20]

% Map to Norton's notation
a = L2;
b = L3;
c = L4;
d = L1;

% --- INPUT VALUES ---
% ค่ามุมเริ่มต้น (เหมือนกันทั้ง Q1 และ Q2)
q2d = 19.94;           [cite_start]% Input angle in degrees [cite: 19]
q2 = deg2rad(q2d);     % Convert to radians

% *** เลือกค่า Input ตรงนี้ ***
% กรณีเช็คคำตอบ Question 1 (Snapshot):
w2 = -2.2;             [cite_start]% Input Velocity (rad/s) [cite: 19]
alpha2 = -0.8;         [cite_start]% Input Acceleration (rad/s^2) [cite: 19]

% กรณีเริ่ม Simulation Question 2 (Initial Conditions):
% w2 = 0.03;           [cite_start]% [cite: 23]
% alpha2 = 0.003;      [cite_start]% [cite: 23]

% --- 2. POSITION ANALYSIS ---
K1 = d/a;
K2 = d/c;
K3 = (a^2-b^2+c^2+d^2)/(2*a*c);
K4 = d/b;
K5 = (c^2 -d^2 -a^2 -b^2)/(2*a*b);

A = cos(q2) - K1 - K2*cos(q2) + K3;
B = -2*sin(q2);
C = K1-(K2+1)*cos(q2) + K3;
D = cos(q2) - K1 + K4*cos(q2) + K5;
E = -2*sin(q2);
F = K1 +(K4-1)*cos(q2)+K5;

% Calculate Theta 4
q41 = 2*atan((-B+sqrt(B^2-4*A*C))/(2*A));
q42 = 2*atan((-B-sqrt(B^2-4*A*C))/(2*A)); % Open Circuit

q41d = rad2deg(q41);
q42d = rad2deg(q42);

% Calculate Theta 3
q31 = 2*atan((-E+sqrt(E^2-4*D*F))/(2*D));
q32 = 2*atan((-E-sqrt(E^2-4*D*F))/(2*D)); % Open Circuit

q31d = rad2deg(q31);
q32d = rad2deg(q32);

% Vector Calculation for Plotting
RA = a*exp(1j*q2);
RBA2 = b*exp(1j*q32); % Using Open Circuit
RB2 = RA + RBA2;
RO4O2 = d*exp(1j*0);
RBO42 = c*exp(1j*q42);

% Extract Components
RAx = real(RA); RAy = imag(RA);
RBA2x = real(RBA2); RBA2y = imag(RBA2);
RB2x = real(RB2); RB2y = imag(RB2);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);
RBO42x = real(RBO42); RBO42y = imag(RBO42);

% Plot Position (Scaled for visibility)
figure(1)
quiver(0,0,RAx,RAy,0,'red','MaxHeadSize',0.5,'LineWidth',2); hold on
quiver(RAx,RAy,RBA2x,RBA2y,0,'blue','MaxHeadSize',0.5,'LineWidth',2);
quiver(0,0,RB2x,RB2y,0,'green','MaxHeadSize',0.5,'LineWidth',2); % Check vector
quiver(0,0,RO4O2x,RO4O2y,0,'black','MaxHeadSize',0.5,'LineWidth',2); 
quiver(RO4O2x,RO4O2y,RBO42x,RBO42y,0,'black','MaxHeadSize',0.5,'LineWidth',2); 
axis equal;
title('Position Analysis');
grid on;

% --- 3. VELOCITY ANALYSIS ---
% Analytical Solution for Open Circuit
w42 = (a*w2*sin(q2-q32))/(c*sin(q42-q32)); 
w32 = (a*w2*sin(q42-q2))/(b*sin(q32-q42));

fprintf('Omega 3: %.4f rad/s\n', w32);
fprintf('Omega 4: %.4f rad/s\n', w42);

% Velocity Vectors
VA = 1j*a*w2*exp(1j*q2);
VBA2 = 1j*b*w32*exp(1j*q32);
VB = VA + VBA2;
VB2 = 1j*c*w42*exp(1j*q42); % Verification (Should match VB)

VAx = real(VA); VAy = imag(VA);
VBA2x = real(VBA2); VBA2y = imag(VBA2);
VBx = real(VB); VBy = imag(VB);
VB2x = real(VB2); VB2y = imag(VB2);

% Plot Velocity
figure(2)
scale_v = 1; % Adjust scale if arrows are too big/small
quiver(RAx,RAy,VAx*scale_v,VAy*scale_v,0,'red','MaxHeadSize',0.5,'LineWidth',2); hold on
quiver(RB2x,RB2y,VBA2x*scale_v,VBA2y*scale_v,0,'blue','MaxHeadSize',0.5,'LineWidth',2); 
quiver(RB2x,RB2y,VBx*scale_v,VBy*scale_v,0,'green','MaxHeadSize',0.5,'LineWidth',2); 
quiver(RB2x,RB2y,VB2x*scale_v,VB2y*scale_v,0,'black','MaxHeadSize',0.5,'LineWidth',2); 
axis equal;
title('Velocity Analysis');
grid on;

% --- 4. ACCELERATION ANALYSIS ---
% Solving Linear System for Alphas (Analytical)
% Ax = B  => [C D; A B] * [alpha4; alpha3] = ... (Standard Matrix Form is safer)

% Using your derived logic (Check coefficients carefully):
% Your Code: alpha42 = (C*E-B*F)/(A*E-B*D);
% Let's match coefficients to standard Ax = B form to be sure.
% Eq X: -b*sin(q3)*alp3 + c*sin(q4)*alp4 = ...
% Eq Y:  b*cos(q3)*alp3 - c*cos(q4)*alp4 = ...

% Using Matrix method (More robust for Simulink function later)
J_mat = [-b*sin(q32), c*sin(q42);
          b*cos(q32), -c*cos(q42)];
      
RHS_acc = [ a*alpha2*sin(q2) + a*w2^2*cos(q2) + b*w32^2*cos(q32) - c*w42^2*cos(q42);
           -a*alpha2*cos(q2) + a*w2^2*sin(q2) + b*w32^2*sin(q32) - c*w42^2*sin(q42)];
       
alp_sol = J_mat \ RHS_acc;
alpha32 = alp_sol(1);
alpha42 = alp_sol(2);

fprintf('Alpha 3: %.4f rad/s^2\n', alpha32);
fprintf('Alpha 4: %.4f rad/s^2\n', alpha42);

% Acceleration Vectors
AA = a*alpha2*1j*exp(1j*q2) - a*w2^2*exp(1j*q2);
ABA = b*alpha32*1j*exp(1j*q32) - b*w32^2*exp(1j*q32);
AB = c*alpha42*1j*exp(1j*q42) - c*w42^2*exp(1j*q42);

AAx = real(AA); AAy = imag(AA);
ABAx = real(ABA); ABAy = imag(ABA);
ABx = real(AB); ABy = imag(AB);

% Plot Acceleration
figure(3)
scale_a = 0.5; % Adjust scale for visibility
quiver(RAx,RAy,AAx*scale_a,AAy*scale_a,0,'red','MaxHeadSize',0.5,'LineWidth',2); hold on
quiver(RB2x,RB2y,ABAx*scale_a,ABAy*scale_a,0,'blue','MaxHeadSize',0.5,'LineWidth',2); 
quiver(RB2x,RB2y,ABx*scale_a,ABy*scale_a,0,'black','MaxHeadSize',0.5,'LineWidth',2); 
axis equal;
title('Acceleration Analysis');
grid on;
