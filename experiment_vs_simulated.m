%% Simulation Over Experimental Data
clear all;
close all;
% fetch variables of systems
run('FeedbackGain');
%% time step for simulation
step_size = 0.001;          % time step
time = 30;                      % time lenght of the simulation
%% initial condition:
theta_start = 0.6781;            % [rad]
phi_start = 0;%-1.492;             % [rad]
theta_dot_start = 0.0;      % [rad/s]
phi_dot_start = 0.0;       % [rad/s]
state = 0;

%% Simulation constants
Kp = 58;
Kd = 12.7;
angle_restrict = pi/1.5;
% region_1 = pi/40;
% region_2 = pi/20;
region_1 = pi/50;
region_2 = pi/5;
encoder_resolution = 1/986;
ADC_resolution = 1/4095;

%%
% Saturation
motor_stall_torque = 8;
%
play_sim = 1;
% run simulations
sim('NonlinearModel_NonlinearController');

%Retrieve Data from simulation
q1_sim = get(q1_angle,'Data');
q2_sim = get(q2_angle,'Data');
timestep  = 1:1:size(q1_sim);

%% Fetch Data from .csv file
q1_wd = 0.906702*2*pi;
q2_wd = 1.08128*2*pi;


% file name for q1 from the 
file_name_q1 = 'C:\Users\Henry\Desktop\Skripsie\Feedback-Control-of-Robotic-Gymnast-MCU\q1_response2.csv';
% file name for q2
file_name_q2 = 'C:\Users\Henry\Desktop\Skripsie\Feedback-Control-of-Robotic-Gymnast-MCU\q2_response2.csv';

% time increments
time_steps = 1e-3;

% response information
system_info_q1 = csvread(file_name_q1,2);
system_info_q2 = csvread(file_name_q2,2);

% time for q1
time_q1 = system_info_q1(1:end,1)./1000;
% time for q2
time_q2 = system_info_q2(1:end,1)./1000;

% q1 response
q1 = (system_info_q1(1:end,2)-2395).*0.00153;

% q2 response
q2 = system_info_q2(1:end,3).*0.01570796326/2;

tau = system_info_q1(1:end,4);

%% Data Cleaning
% Removing points outside boundary:
i = find(q1(:,1) > 0.6*pi);
j = find(q1(:,1) < -0.6*pi);
q1(i) = 0;
q1(j) = 0;

% Removing points outside boundary:
i = find(q2(:,1) > 2*pi);
j = find(q2(:,1) < -2*pi);
q2(i) = 0;
q2(j) = 0;

% smoothing function to remove any noise
q1 = smooth(q1);
q2 = smooth(q2);






% theta angle
figure(1)
plot(time_q1,q1.*180/pi,'-b','LineWidth',2);
grid on
hold on
plot(timestep.*step_size,q1_sim.*180/pi, '-r', 'LineWidth',2)
title('Measured Response vs Simulated Response of Initial Condition Test  with $$\phi = 0$$ ','Interpreter','latex','FontSize',12)
yticks([-2*pi -pi 0 pi 2*pi]);
ylabel('$\theta$ [Degrees]','Interpreter','latex','FontSize',12);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
yticklabels({'-50','-40','-30','-20','-10','0','10','20','30','40','50'})
yticks([-50 -40 -30 -20 -10 0 10 20 30 40 50]);
legend({'Measured Response','Simulated Response'},'Interpreter','latex','FontSize',12)




% \phi angle 
figure(2)
plot(time_q2,q2.*180/pi,'-r','LineWidth',2);
grid on
hold on
plot(timestep.*step_size,q2_sim.*180/pi, '-b', 'LineWidth',2);
title('Measured Response vs Simulated Response of Initial Condition Test  with $$\theta = 0$$','Interpreter','latex','FontSize',12)
ylabel('$\phi$ [rad]','Interpreter','latex','FontSize',12);
xlabel('time [s]','Interpreter','latex','FontSize',12);
yticklabels({'-90','-80','-70','-60','-50','-40','-30','-20','-10','0','10','20','30','40','50','60','70','80','90'})
yticks([-90 -80 -70 -60 -50 -40 -30 -20 -10 0 10 20 30 40 50 60 70 80 90]);
legend({'Measured Response','Simulated Response'},'Interpreter','latex','FontSize',12)


%% FFT for q1

figure(3)
Fs = 250;          % sampling rate
T = 1/Fs;
L_q1 = length(time_q1);         % number of samples taken
time_q1=time_q1./1000;      % convert time_q1 from [ms] to [s]
Y_q1 = fft(q1);                     % FFT of response
P2_q1 = abs(Y_q1);              % absolute value of FFT 
P1_q1 = P2_q1(1:L_q1/2+1);
P1_q1(2:end-1) = 2*P1_q1(2:end-1);
f_q1 = Fs*(0:(L_q1/2))/L_q1;

L_q2 = length(time_q2);
time_q2=time_q2./1000;
Y_q2 = fft(q2);
P2_q2 = abs(Y_q2);
P1_q2 = P2_q2(1:L_q2/2+1);
P1_q2(2:end-1) = 2*P1_q2(2:end-1);
f_q2 = Fs*(0:(L_q2/2))/L_q2;
plot(f_q1(1:60),P1_q1(1:60),'-*r', 'LineWidth',2) 
grid on
hold on
plot(f_q2(1:20),P1_q2(1:20),'-*b', 'LineWidth',2)
title('Frequency Content of Initial Condition System Responses','Interpreter','latex','FontSize',12)
ylabel('Magnitude ','Interpreter','latex','FontSize',12);
xlabel('Frequency [Hz]','Interpreter','latex','FontSize',12);

Fs = 1/step_size;          % sampling rate
T = 1/Fs;
L_q1 = length(timestep.*step_size);         % number of samples taken
time_q1=timestep.*step_size;      % convert time_q1 from [ms] to [s]
Y_q1 = fft(q1_sim);                     % FFT of response
P2_q1 = abs(Y_q1);              % absolute value of FFT 
P1_q1 = P2_q1(1:L_q1/2+1);
P1_q1(2:end-1) = 2*P1_q1(2:end-1);
f_q1 = Fs*(0:(L_q1/2))/L_q1;

L_q2 = length(timestep.*step_size);
time_q2=timestep.*step_size;
Y_q2 = fft(q2_sim);
P2_q2 = abs(Y_q2);
P1_q2 = P2_q2(1:L_q2/2+1);
P1_q2(2:end-1) = 2*P1_q2(2:end-1);
f_q2 = Fs*(0:(L_q2/2))/L_q2;
plot(f_q1(1:60),P1_q1(1:60),'-*r', 'LineWidth',2) 
plot(f_q2(1:20),P1_q2(1:20),'-*b', 'LineWidth',2)