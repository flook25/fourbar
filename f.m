clear all;
close all;
clc;

L1 = 0.1; %length of Link1 d
L2 = 0.04; %length of Link2 a
L3 = 0.12; %length of Link3 c
L4 = 0.08; %length of Link4 d
a = L2;
b = L3;
c = L4;
d = L1;


q2d = 40;
q2 = deg2rad(40); % radian angle

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

q41 = 2*atan((-B+sqrt(B^2-4*A*C))/(2*A));
q42 = 2*atan((-B-sqrt(B^2-4*A*C))/(2*A));

q41d = rad2deg(q41) + 360;
q42d = rad2deg(q42);

q31 = 2*atan((-E+sqrt(E^2-4*D*F))/(2*D));
q32 = 2*atan((-E-sqrt(E^2-4*D*F))/(2*D));

q31d = rad2deg(q31) + 360; 
q32d = rad2deg(q32);

RA = a*exp(j*q2);
RBA1 = b*exp(j*q31); %q31 Crossed circuit
RBA2 = b*exp(j*q32); %q32 Open circuit

RB1 = RA + RBA1; %Position vector for Crossed circuit
RB2 = RA + RBA2; %Position vector for Open circuit

RAx = real(RA);
RAy = imag(RA);

RBA1x = real(RBA1);
RBA1y = imag(RBA1);

RBA2x = real(RBA2);
RBA2y = imag(RBA2);
RB1x = real(RB1); %Position Bx for Crossed circuit
RB1y = imag(RB1); %Position By for Crossed circuit
RB2x = real(RB2); %Position Bx for Open circuit
RB2y = imag(RB2); %Position By for Open circuit

RO4O2 = d*exp(j*0);
RBO41 = c*exp(j*q41);
RBO42 = c*exp(j*q42);


RB3 = RO4O2 + RBO41; %crossed circuit
RB4 = RO4O2 + RBO42; %open circuit
RB3x = real(RB3);
RB3y = imag(RB3);
RB4x = real(RB4);
RB4y = imag(RB4);

RO4O2x = real(RO4O2);
RO4O2y = imag(RO4O2);
RBO42x = real(RBO42);
RBO42y = imag(RBO42);

quiver(0,0,RAx,RAy,0,'red','MaxHeadSize',0.5,'LineWidth',2); hold on
quiver(RAx,RAy,RBA2x,RBA2y,0,'blue','MaxHeadSize',0.5,'LineWidth',2);
quiver(0,0,RB2x,RB2y,0,'green','MaxHeadSize',0.5,'LineWidth',2);
quiver(0,0,RO4O2x,RO4O2y,0,'black','MaxHeadSize',0.5,'LineWidth',2); 
quiver(RO4O2x,RO4O2y,RBO42x,RBO42y,0,'black','MaxHeadSize',0.5,'LineWidth',2); 
axis equal;



