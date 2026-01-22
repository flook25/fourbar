clear all;
close all;
clc;

% --- 1. SYSTEM PARAMETERS ---
L1 = 210; % Length of Link1 d (Ground)
L2 = 180; % Length of Link2 a (Green Link)
L3 = 180; % Length of Link3 b (Yellow Link)
L4 = 118; % Length of Link4 c (Grey Link)

a = L2;
b = L3;
c = L4;
d = L1;

% Input Angle (จาก Loop ก่อนหน้า Green Link = -30.69)
q2d = -30.69;
q2 = deg2rad(q2d); % radian angle

% --- 2. CALCULATION (Professor's Pattern) ---
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

% Calculate Theta 4 (Output - Grey)
q41 = 2*atan((-B+sqrt(B^2-4*A*C))/(2*A));
q42 = 2*atan((-B-sqrt(B^2-4*A*C))/(2*A));

q41d = rad2deg(q41);
q42d = rad2deg(q42);

% Calculate Theta 3 (Coupler - Yellow)
q31 = 2*atan((-E+sqrt(E^2-4*D*F))/(2*D));
q32 = 2*atan((-E-sqrt(E^2-4*D*F))/(2*D));

q31d = rad2deg(q31); 
q32d = rad2deg(q32);

% --- 3. VECTOR DEFINITION ---
RA = a*exp(j*q2);

% เลือก Set คำตอบที่ต้องการ Plot (ในที่นี้เลือก Set 2 ตามตัวอย่างอาจารย์)
RBA1 = b*exp(j*q31); %q31
RBA2 = b*exp(j*q32); %q32 (เลือกใช้ตัวนี้ตาม Code ตัวอย่าง)

RB1 = RA + RBA1; 
RB2 = RA + RBA2; %Position vector B

% แยก Component
RAx = real(RA);
RAy = imag(RA);

RBA2x = real(RBA2);
RBA2y = imag(RBA2);

RB2x = real(RB2); 
RB2y = imag(RB2); 

RO4O2 = d*exp(j*0);
RBO42 = c*exp(j*q42); % Vector ของ Link 4 (Grey)

RO4O2x = real(RO4O2);
RO4O2y = imag(RO4O2);
RBO42x = real(RBO42);
RBO42y = imag(RBO42);

% --- 4. PLOTTING (เปลี่ยนสีตามก้านจริง) ---
figure; hold on;

% Link 2 (Green) - วาด RA เป็นสีเขียว
quiver(0,0,RAx,RAy,0,'g','MaxHeadSize',0.5,'LineWidth',3); 

% Link 3 (Yellow) - วาด RBA ต่อจาก RA เป็นสีเหลือง
quiver(RAx,RAy,RBA2x,RBA2y,0,'y','MaxHeadSize',0.5,'LineWidth',3);

% Resultant Vector (RB2) - วาดเวกเตอร์ลัพธ์ (เส้นประสีแดง)
quiver(0,0,RB2x,RB2y,0,'r','MaxHeadSize',0.5,'LineWidth',1, 'LineStyle', '--');

% Ground (L1) - สีดำ
quiver(0,0,RO4O2x,RO4O2y,0,'k','MaxHeadSize',0.5,'LineWidth',2); 

% Link 4 (Grey) - วาดจาก O4 ไปชนจุด B (สีเทา)
quiver(RO4O2x,RO4O2y,RBO42x,RBO42y,0,'Color',[0.5 0.5 0.5],'MaxHeadSize',0.5,'LineWidth',3); 

axis equal;
grid on;
