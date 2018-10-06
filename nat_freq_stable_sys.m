%% nat_freq_stable_sys
% Determines the simulated natural frequency 
% Determines the simulated natural frequency of 
% the system around the stable equilibrium position

close all;
clear all;
run('system_variables');

% m1 = 1;
% m2 = 1;
% L1 = 1;
% L2 = 1;
% l1 = 0.5;
% l2 = 1;
% Ia = m1*L1^2/12;
% Ib = m1*L1^2/12;
% g=9.81;
% f1 =0 ;
% f2 =0;



A = (m1*l1^2+ m2*l2^2 + m2*L1^2 + 2*m2*l2*L1 + Ia + Ib); % theta dotdot
B = (m2*l2^2 + m2*L1*l2+ Ib);              % phi dotdot
C = (m1*g*l1 + m2*g*L1 + m2*g*l2);         % theta
D = (m2*g*l2);                         % phi
E =  f1;                               % theta dot

F = (m2*l2^2 + m2*L1*l2 + Ib);              % theta dotdot
G = (m2*l2^2 + Ib);                       % phi dotdot
H = (m2*g*l2);                         % theta 
I = (m2*g*l2);                         % phi
J = f2;                               % phi dot
K = -f2;                              % theta dot


MM = [A B; F G];        % q_dotdot
CC = [E 0; K J];           % q_dot
KK = [C D; H I];           % q
%%

[X,e,s]= polyeig(KK,CC,MM);
disp('Eigenvalues from Quadratic Eigenvalue problem:');
disp(e);


%%
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
  %%
A = [   0,0,1,0;
        0,0,0,1;
        -DD/AA,-EE/AA,-BB/AA,-CC/AA;
        -II/FF,-JJ/FF,-GG/FF,-HH/FF
     ]

 
B = [0;0;1/AA;1/FF] 
C = eye(4);
D = [0;0;0;0];

eigen_vals = eig(A);
format long
disp('Eigenvalues from ordinary eigenvalue problem:')
disp(eigen_vals)

