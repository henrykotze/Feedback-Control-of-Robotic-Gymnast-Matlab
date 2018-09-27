%% Linearized Model Of the Double Pendulum
% Script will determine the A,B,C,D matrix, determine the feedback gain K
% matrix provided desired poles. Will output the reponse of the system
% for a given start conditions 

clear all;
run('system_variables');

A = (Ia+Ib+m2*l2^2+m2*L1^2+2*m2*l2*L1); % theta dotdot
B = (Ib+m2*l2^2+m2*L1*l2);              % phi dotdot
C = (-m1*g*l1-m2*g*L1-m2*g*l2);         % theta
D = (-m2*g*l2);                         % phi
E =f1;                               % theta dot

F = (Ib+m2*l2^2+m2*L1*l2);              % theta dotdot
G = (Ib+m2*l2^2);                       % phi dotdot
H = (-m2*g*l2);                         % theta 
I = (-m2*g*l2);                         % phi
J = f2;                               % phi dot
K = -f2;                              % theta dot

AA = (F-A*G/B);     % theta dotdot
BB = (-E*G/B-K);    % theta dot
CC = (J);        % phi dot
DD = (H-C*G/B);     % theta
EE = (-D*G/B+I);    % phi

FF = (G-B*F/A);     % phi dotdot
GG = (-K-E*F/A);     % theta dot 
HH = (J);           % phi dot
II = (-C*F/A+H);    % theta
JJ = (I-D*F/A);     % phi


 %  A = [ theta
 %        phi
 %        theta dot
 %        phi dot
 %         ]       
  
A_ = [   0,0,1,0;
        0,0,0,1;
        -DD/AA,-EE/AA,-BB/AA,-CC/AA;
        -II/FF,-JJ/FF,-GG/FF,-HH/FF
     ]
B = [0;0;1/AA;1/FF] ;
C = eye(4);
D = [0;0;0;0];

P = eig(A_)
% 
sys = ss(A_,B,C,D);
% tfsys = tf(sys);
% pzmap(sys);
poles_system = pole(sys);

% poles of system
disp('Poles of the system is:');
disp(poles_system);

%% 4th order Bessel polynomials
% -4.0156+-j5.0723,-5.5281+-j3.0813;
% pc = [-4.0156-5.0723i;-4.0156+5.0723i;-5.5281-3.0813i;-5.5281+3.0813i]./0.5;
% All sclea to give settling time of 1 second, which you can change to t_s
% by dividing the poles by t_s

%% ITAE Characteristic equation table
% s^4 + 2.41*w*s^3 + 4.93*w^2*s^2 + 5.14*w^3*s + w^4
% pc = [-0.4240+1.2630i;-0.4240-1.260i;-0.6260+0.4141i;-0.6260-0.4141i].*15;

%% Dominant Poles
% 
pc = [    -8.8208
   -4.3594
    -8.8208
    -4.3594
];
K = acker(A_,B,pc)
% pzmap(new_sys)

%% Simulating Linear Controller on Linearised System
 q_start = [pi/80;-pi/60;0;0];
 motor_stall_torque = 2.8;
% initial(sys,q_start)
% 
 A = A_;
time = 2.5;
time_steps = 0.0001;
working_points = [-pi;0;0;0];
sim('LinearModelAndLinearController');


%% Figure 1
figure(1)
plot(q,'LineWidth',2);
% labels Numbers and ticks
set(gca,'FontName','Times','FontSize',12)
% Title Information
title_text1 = title('Response of Linear System with starting conditions of $q_{0}$');
set(title_text1,'Interpreter','latex');
set(title_text1,'FontSize',12);

% Label Information
xlabel('Time - [s]','Interpreter','latex','FontSize',12);
ylabel('Anguler Position -[rad] or Angular Velocity [rad/s]','Interpreter','latex','FontSize',12);
text(time-0.5,320,'$q_{0}$ = $[\pi+\frac{\pi}{8},0,0,0]$','Interpreter','latex','FontSize',12);

% Legend Information
legend({'$\theta$','$\phi$','$\dot{\theta}$','$\dot{\phi}$'},'Interpreter','latex','FontSize',12);
grid on;

q1 = getsamples(q,1); 
q2 = getsamples(q,2);

%% Figure 2
figure(2)
plot(tau);
title('Torque During Response with starting conditions $q_{0}$','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$\tau$ [N\\m]','Interpreter','latex','FontSize',12);
text(time-0.5,pi, '$q_{0}$ = $[\phi+\frac{pi}{8},0,0,0]$','Interpreter','latex','FontSize',12);
grid on;