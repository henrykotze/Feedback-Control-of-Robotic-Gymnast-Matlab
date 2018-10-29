%% Variables for controller on MCU
syms q1
syms q2 
syms q1dot
syms q2dot
syms Kp
syms Kd
syms angle_restriction
syms q2_d


g = 9.81;
Ia = 0.0022;
Ib = 0.0043;
L1 = 0.2350;
L2 = 0.3100;
l1 = 0.2056;
l2 = 0.1940;
m1 = 0.5763;
m2 = 0.5639;
f1 = 0.025*sign(q1dot);
f2 = 0.014*(q2dot);


d11 = m1*l1^2 + m2*L1^2 + m2*l2^2 + 2*m2*l2*L1*cos(q2) + Ia + Ib;
d12 = m2*l2^2 + m2*L1*l2*cos(q2) + Ib;
h1 = -m2*L1*l2*q2dot^2*sin(q2)-2*m2*l2*L1*q1dot*q2dot*sin(q2);
psi1 =m1*g*l1*sin(q1)  + m2*g*L1*sin(q1)+ m2*g*l2*sin(q1+q2);
f1 = 0.025*sign(q1dot);
d21 = m2*l2^2 + m2*L1*l2*cos(q2) + Ib;
d22 = m2*l2^2 + Ib;
h2= m2*L1*l2*q1dot^2*sin(q2);
psi2= m2*g*l2*sin(q1+q2);
f2 = 0.014*(q2dot);

d = d22 - d21*d12/d11;
h = h2 - d21*h1/d11;
psi = psi2 - d21*psi1/d11; 
f = f2 - d21*f1/d11;




%% Controller 
%         v2 = Kp*(q2_d-q2)-Kd*q2dot;
        v2 = 1*(Kp*(angle_restriction*atan(q1dot)-q2)-Kd*q2dot);
%         tau = zeta - H*alpha/A + (I-B*H/A)*v2;
        tau = d*v2 + h + psi+f;
        
        
 vpa(tau,5)