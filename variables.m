%% Constants for systems

g = 9.81;                          %      [kg/s^2] gravity 
L1 = 0.235;                       %     [m] Total lenght of non actuated pendulum 
L2 = 0.245;                       %     [m] Total lenght of actuated pendulum 


% density of iron
p2 = 7874;                                  % kg/m^3
volume_p2 = 0.01*0.01*L2;           % [m^3] Volume of actuaded pendulum
m2 = p2*volume_p2;                     % [kg] Mass of pendulum 2
Ib = 1/12*m2*(0.01^2+L2^2)+m2*(L1/2)^2; % Inertia of actuated pendulum


% density of Aluminium
p1 = 2700; % kg/m^3
% Volume of Cylinder
volume_cyl = L1*0.012*0.012;
% mass of cylinder
m_cylinder = volume_cyl*p1;
% Moment of Inertia of cylinder
I_cyl = 1/12*m_cylinder*(0.012^2+L1^2);


% mass of motor
motor_mass = 242e-3+190e-3; % [kg] motor mass + gearbox mass [kg]

m1 = m_cylinder + motor_mass;  % [kg] mass of pendulum 1
l1 = L1;                            % [m] lenght to mass middle point of pendulum 1 from shoulder
l2 = L2/2;                         % [m] lenght to mass middle point of pendulum 2 from elbow

Ia = motor_mass*L1^2+I_cyl;       %  Inertia of Pendulum 1 from shoulder

