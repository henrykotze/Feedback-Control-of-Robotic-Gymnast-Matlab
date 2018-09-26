%% Constants for systems

g = 9.81;                                      % [kg/s^2] gravity 
L1 = 0.235;                                   % [m] Total lenght of non actuated pendulum 
L2 = 0.245;                                   % [m] Total lenght of actuated pendulum 
point_mass = 242e-3+190e-3;      % [kg] motor mass + gearbox mass
l1 = L1;                                         % [m] lenght to mass middle point of pendulum 1 from shoulder
l2 = L2/2;                                      % [m] lenght to mass middle point of pendulum 2 from elbow

density_p2 = 7874;                         % [kg/m^3] density of iron
volume_p2 = 0.01*0.01*L2;             % [m^3] Volume of actuaded pendulum
m2 = density_p2*volume_p2;          % [kg] Mass of pendulum 2

% Inertia of actuated pendulum from elbow
Ib = 1/12*m2*(0.01^2+L2^2)+m2*(l2)^2; 

% inertia of actuated pendulum from shoulder
ib2 = 1/12*m2*(0.01^2+L2^2)  + m2*(L1+l2)^2; 



density_p1 = 2700;                          % [kg/m^3] density of Aluminium
volume_p1 = L1*0.012*0.012;          % [m^3] Volume of non-actuated pendulum
m_p1 = volume_p1*density_p1;        % [kg] mass of non-actuated pendulum

% Moment of Inertia of non-actuated pendulum from shoulder
I_p2 = 1/12*m_p1*(0.012^2+L1^2) + m_p1*(l1)^2;

m1 = m_p1 + point_mass;               % [kg] total mass of non actuated pendulum + pointmass

Ia = point_mass*L1^2+I_p2;             %  Inertia of  non actuated pendulum

