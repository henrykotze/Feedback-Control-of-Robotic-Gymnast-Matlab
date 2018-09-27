%% System variables

clear all;
close all;

% gravity 
g = 9.81;

% Lenght of non-actuated pendulum [kg]
L1 = 0.235;

% Centriod of non-acuated pendulum [m]
l1 = 0.235;

% Lenght of actuated pendulum [kg]
L2 = 0.245;

% Centriod of actuated pendulum [m]
l2 = L2;

% mass of cylindrical square in non-actuated pendulum [kg]
m_cyl = 0.1443;

% mass of motor and gearbox [kg]
pointmass_1 = 242e-3+190e-3;

% mass of non-actuated pendulum
m1 = pointmass_1 + m_cyl;

% mass of actuated iron square [kg]
m_iron = 0.18585;

% mass of point mass on actuated pendulum [kg]
pointmass_2 = 0.30692;

% mass of actuated pendulum [kg]
m2 = m_iron + pointmass_2;

% Inertia of cylindrical square from shoulder
I_cyl = 1/12*m_cyl*(0.012^2+L1^2) + m_cyl*(L1/2)^2;

% Inertia of pointmass_1 from shouder
I_pointmass_1 = pointmass_1*L1^2;

% Inertia of non-actuated pendulum from shoulder
Ia = I_cyl + I_pointmass_1;

% Inertia of iron square from elbow
I_iron = 1/12*m_iron*(0.01^2+L2^2)+m_iron*(L2/2)^2;

% Inertia of point mass 1 from elbow
I_pointmass_2 = pointmass_2*L2^2;

% Inertia of actuated pendulum around elbow
Ib = I_pointmass_2 + I_iron;

% Friction coefficient on  non-actuated pendulum
f1 = 0;

% Friction coefficient on actuated pendulum
f2 = 0;

% format long
format short

disp('Ia');disp(Ia);
disp('Ib');disp(Ib);
disp('L1');disp(L1);
disp('L2');disp(L2);
disp('l1');disp(l1);
disp('l2');disp(l2);
disp('m1');disp(m1);
disp('m2');disp(m2);
disp('f1');disp(f1);
disp('f2');disp(f2);








