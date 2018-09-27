%% FeedbackGain
% Calculate the feedback gain matrix, K, for given desired poles for 
% system and determeine the natural frequency of the system
close all;
clear all;
run('system_variables');



% Desired Poles: Bessel Prototype
pc = [    -8.8208
   -4.3594
    -8.8208
    -4.3594
];

% % test variables from articl
% m1 = 1;
% m2 = 1;
% L1 = 1;
% L2 = 2;
% l1 = 0.5;
% l2 = 1;
% Ia = 0.0833;
% Ib = 0.333;
% g=9.8;

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

K = acker(A,B,pc);
disp('K Matrix:')
disp(K);

eigen_vals = eig(A);
disp('Eigenvalues from ordinary eigenvalue problem:')
disp(eigen_vals)