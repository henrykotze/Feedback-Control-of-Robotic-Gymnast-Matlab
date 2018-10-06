%% Variables for controller on MCU

g = 9.81;
Ia = 0.0265152541;
Ib = 0.0221429705;
L1 = 0.2350;
L2 = 0.245;
l1 = 0.235;
l2 = 0.245;
m1 = 0.5763;
m2 = 0.49277;
f1 = 1.0953e-04;
f2 = 0.0028;
syms q1
syms q2 
syms q1dot
syms q2dot
syms Kp
syms Kd
syms angle_restriction

A = Ia+Ib+m2*L1^2+m2*l2^2+2*m2*l2*L1*cos(q2);
B = Ib+m2*l2^2+m2*L1*l2*cos(q2);
E = m1*g*l1*sin(q1);
F = m2*g*L1*sin(q1);
G = m2*g*l2*sin(q1+q2);
C = 2*m2*l2*L1*q1dot*q2dot*sin(q2);
D = m2*L1*l2*q2dot^2*sin(q2);

H =Ib+m2*l2^2+m2*L1*l2*cos(q2);
I =Ib+m2*l2^2;
K = m2*L1*l2*q1dot^2*sin(q2);
J = m2*L1*l2*q1dot*q2dot*sin(q2);
M = m2*g*l2*sin(q1+q2);
L = m2*L1*l2*q1dot*q2dot*sin(q2);
% Coefficient for Damping 

Z = -f1*q1dot;
Y = -f2*(q2dot-q1dot);

%% Collocated Linearization
alpha = -C-D+E+F+G-Z;
zeta = -J+K+L+M-Y;
angle_restriction = pi/10;
Kp = 58;
Kd = 12.7;
%% Controller 
v2 = 1*(Kp*(angle_restriction*atan(q1dot)-q2)-Kd*q2dot);
        tau = zeta - H*alpha/A + (I-B*H/A)*v2