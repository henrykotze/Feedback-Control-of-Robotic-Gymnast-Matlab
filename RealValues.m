%% Real Constants for Simulations

% density of iron
p2 = 7870; % kg/m^3
volume_p2 = 0.01*0.01*0.25; %   [m^3] volume of upper pendulum

% density of Aluminium
p1 = 2700; % kg/m^3

% Moment of Inertia of cylinder
I_cyl = 1/12*pi*p1*0.25*(3*(0.006^4-0.005^4)+0.25^2*(0.006^2-0.005^2))

% Volume of Cylinder
volume_cyl = pi*0.25*(0.006^2-0.005^2);

% mass of motor
motor_mass = 0.205; % [kg] motor mass 



m1 = volume_cyl*p1 + motor_mass;  % [kg] mass of pendulum 1
g = 9.81;                       % [mm/s^2] gravity 
L1 = 0.25;                       % [mm] Total lenght of pendulum 1
L2 = 0.25;                       % [mm] Total lenght of pendulum 2
m2 = p2*volume_p2;               % [kg] Mass of pendulum 2
l1 = 0.125;                      % [mm] lenght to mass middle point of pendulum 1 from shoulder
l2 = 0.125;                      % [mm] lenght to mass middle point of pendulum 2 from elbow

Ia = motor_mass*L1^2+I_cyl       % [kg mm^4] Inertia of Pendulum 1 from shoulder

Ib = 1/12*m2*(0.01^2+0.01^2)   % [kg mm^4] Inertia of Pendulum 2 from elbow