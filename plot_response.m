%% plot responses
clear all 
close all

% 
torque_constant = 19.1e-3; %[Nm]
gearbox_ratio = 14 ;
motor_driver_ratio =375;
Resistor = 150; % [Ohm]

% file name for q1 from the 
file_name_q1 = 'C:\Users\Henry\Desktop\Skripsie\Feedback-Control-of-Robotic-Gymnast-MCU\test6.csv';
% file name for q2
file_name_q2 = 'C:\Users\Henry\Desktop\Skripsie\Feedback-Control-of-Robotic-Gymnast-MCU\test6.csv';

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
q1 = -1.*(system_info_q1(1:end,2)-2280).*0.00153*(180/pi);

% q2 response
q2 = system_info_q2(1:end,3).*0.01570796326/2*(180/pi);

tau = system_info_q1(1:end,4)./(1*Resistor).*motor_driver_ratio.*torque_constant.*gearbox_ratio;


figure(1)
plot(time_q1,q1,'r-','LineWidth',2);
hold on
title('Experimental Response of Robotic Gymanst with Swing-up Controller','Interpreter','latex','FontSize',12)
ylabel('$\theta$ and $\phi$ [Degrees]','Interpreter','latex','FontSize',12);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
hold on
plot(time_q2,q2,'b.-','LineWidth',2);
legend({'$\theta$','$\phi$'},'Interpreter','latex','FontSize',12)
grid on



figure(2)
plot(time_q1.*1000,tau,'r.-','LineWidth',1);
hold on
title('Torque Delivered during Swing-up of Robotic Gymnast','Interpreter','latex','FontSize',12)
ylabel('$\tau$ [Nm]','Interpreter','latex','FontSize',12);
xlabel('Time [ms]','Interpreter','latex','FontSize',12);
hold on
grid on