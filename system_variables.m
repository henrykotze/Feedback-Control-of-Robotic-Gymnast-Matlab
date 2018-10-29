%% system_variables.m

clear all;
close all;

% gravity 
g = 9.81;

% Lenght of non-actuated pendulum [kg]
L1 = 0.235;

% Lenght of actuated pendulum [m]
L2 = 0.303;

% Lenght to point mass 2
% L2_pointmass = 0.27;
L2_pointmass = 0.263;

% lenght of cylindrical square in non-actuated pendulum [m]
L_cyl = 0.235;

% Lengt of iron rod in actuated pendulum [m]
%L_iron = 0.230;
L_iron = 0.223;

% mass of cylindrical square in non-actuated pendulum [kg]
m_cyl = 0.1443;

% mass of motor and gearbox [kg]
pointmass_1 = 242e-3+190e-3;

% mass of non-actuated pendulum
m1 = pointmass_1 + m_cyl;

% mass of actuated iron square [kg]
% m_iron = 0.18585;
m_iron = 0.257;
% mass of point mass on actuated pendulum [kg]
pointmass_2 = 0.30692;

% mass of actuated pendulum [kg]
m2 = m_iron + pointmass_2;


% Centriod of non-acuated pendulum [m]
l1 = (m_cyl*L_cyl/2+pointmass_1*L_cyl)/(m_cyl+pointmass_1);


% Centriod of actuated pendulum [m]
l2 = (pointmass_2*L2_pointmass + m_iron*L_iron/2)/(m_iron+pointmass_2);


% Inertia of cylindrical square around centroid of cylindrical square
I_cyl = 1/12*m_cyl*(0.012^2+L_cyl^2);

% Inertia of pointmass_1 to centroid of non-actuated pendulum
I_pointmass_1 = pointmass_1*(L1-l1)^2;

% Inertia of non-actuated pendulum from around centroid
Ia = I_cyl + m_cyl*(l1-L_cyl/2)^2 + I_pointmass_1;

% Inertia of iron square from around iron square centroid
I_iron = 1/12*m_iron*(0.01^2+L_iron^2);

% Inertia of point mass 1 from centroid of actuated pendulum
I_pointmass_2 = pointmass_2*(L2_pointmass-l2)^2;

% Inertia of actuated pendulum around elbow
Ib = I_pointmass_2 + I_iron + m_iron*(l2-L_iron/2)^2;

% Friction coefficient on  non-actuated pendulum
f1 = 0.0071;%1.0953e-04;

% Friction coefficient on actuated pendulum
f2 = 0.0075;%.0028;

% format long
format short



g = 9.81;
Ia = 0.0022;
Ib = 0.0043;
L1 = 0.2350;
L2 = 0.3100;
l1 = 0.2056;
l2 = 0.1940;
m1 = 0.5763;
m2 = 0.5639;
f1 = 0.024;
f2 = 0.013;


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





