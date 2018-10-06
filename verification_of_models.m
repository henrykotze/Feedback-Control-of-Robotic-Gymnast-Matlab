
%% 

close all;
clear all;
run('system_variables');

M = m1 + m2;
L = L1 ;

I = Ia + m1*l1^2 + Ib + m2*(L1+l2)^2;

wn = sqrt( ( M*g*L)/I )