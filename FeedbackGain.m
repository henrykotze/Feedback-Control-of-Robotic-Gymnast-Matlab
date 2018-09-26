%% FeedbackGain
% Calculate the feedback gain matrix, K, for given desired poles for 
% system and determeine the natural frequency of the system
close all;
clear all;
run('variables');



% Desired Poles: Bessel Prototype
pc = [    -15.7911;
    -13.6286;
   -6.2262
   -3.8406];


A = (Ia+Ib+m2*l2^2+m2*L1^2+2*m2*l2*L1);     % theta dotdot
B = (Ib+m2*l2^2+m2*L1*l2);              % phi dotdot
C = (-m1*g*l1-m2*g*L1-m2*g*l2);         % theta
D = (-m2*g*l2);                                      % phi
E = 4.3720e-04;                               % theta dot

F = (Ib+m2*l2^2+m2*L1*l2);              % theta dotdot
G = (Ib+m2*l2^2);                       % phi dotdot
H = (-m2*g*l2);                         % theta 
I = (-m2*g*l2);                         % phi
J = 1.9;                               % phi dot
K = -1.9;                        % theta dot


MM = [A B; F G];        % mass matrix
CC = [E 0; K J];           % damping matrix
KK = [C D; H I];           % stiffness matrix

[X,e,s]= polyeig(KK,CC,MM);
disp('Eigenvalues from Quadratic Eigenvalue problem:');
disp(e);



%%




AA = (F-A*G/B);     % theta dotdot
BB = (-E*G/B+K);    % theta dot
CC = (J);           % phi dot
DD = (H-C*G/B);     % theta
EE = (-D*G/B+I);    % phi

FF = (G-B*F/A);     % phi dotdot
GG = (K-E*F/A);     % theta dot 
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
     ];

 
B = [0;0;1/AA;1/FF] ;
C = eye(4);
D = [0;0;0;0];

K = acker(A,B,pc);
disp('K Matrix:')
disp(K);

eigen_vals = eig(A);
disp('Eigenvalues from ordinary eigenvalue problem:')
disp(eigen_vals)