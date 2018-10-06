%% Determine zeta's for response
% Script allows you to select the peaks of the response and assuming a
% exponential decaying function it will determine the best zeta values
% based on linear regression
close all;
clear all;
% Requires natural frequency for responses, determine in data_aquisition.m
q1_wd = 3.62902*2*pi;
q2_wd = 4.03*2*pi;


% file name for q1 from the 
file_name_q1 = 'C:\Users\Henry\Desktop\Skripsie\Feedback-Control-of-Robotic-Gymnast-MCU\q1_response1.csv';
% file name for q2
file_name_q2 = 'C:\Users\Henry\Desktop\Skripsie\Feedback-Control-of-Robotic-Gymnast-MCU\q2_response5.csv';

% time increments
time_steps = 1e-3;

% response information
system_info_q1 = csvread(file_name_q1,2);
system_info_q2 = csvread(file_name_q2,2);

% time for q1
time_q1 = system_info_q1(1:end,1);
% time for q2
time_q2 = system_info_q2(1:end,1);

% q1 response
q1 = (system_info_q1(1:end,2)-2195).*0.00153;

% q2 response
q2 = system_info_q2(1:end,3).*0.01570796326/2;

tau = system_info_q1(1:end,4);

% three point central difference
q1dot = cent_diff_n(q1,time_steps,3);
q2dot = cent_diff_n(q2,time_steps,3);
q1dotdot = cent_diff_n(q1dot,time_steps,3);
q2dotdot = cent_diff_n(q2dot,time_steps,3);

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

%  Response for q1
% figure(1);
plot(time_q1,q1,'.r');
title('System Response from Initial Conditions with $\phi = 0 $ ','Interpreter','latex','FontSize',12)
yticks([-2*pi -pi 0 pi 2*pi]);
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',12);
xlabel('time [ms]','Interpreter','latex','FontSize',12);
yticklabels({'-1.5\pi','-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi'})
yticks([-1.5*pi -1*pi -0.5*pi 0 0.5*pi 1*pi 1.5*pi]);
grid on
xt = get(gca, 'XTick');
set(gca, 'FontSize', 12);
yt = get(gca, 'YTick');
set(gca, 'FontSize', 12);
hold on

% select 10 first peaks to determine best fit zeta
[x_q11,y_q11] = ginput(8);
fit = polyfit(x_q11(1:end), log(y_q11),1)

zeta_11 = sqrt( 1/(1+ q1_wd^2/fit(1)^2) );
q1_wn = q1_wd/sqrt(1-zeta_11^2);
disp('Zeta value for q1 response:');
disp(zeta_11);
disp('Natural Frequency from q1 response:');


% zeta_11 = fit(1)./(-q1_wn)
A_11 = exp(fit(2))
plot(time_q1, A_11.*exp(-1*zeta_11*q1_wn*time_q1),'-b','LineWidth',2);
legend('Experimental results', 'Approximated decay function');



%%

% Response for q2
figure(2)
plot(time_q2,q2,'b.');
title('System Response from Initial Conditions with $\theta = 0 $ ','Interpreter','latex','FontSize',12)
yticks([-2*pi -pi 0 pi 2*pi]);
ylabel('$\phi$ [rad]','Interpreter','latex','FontSize',12);
xlabel('time [ms]','Interpreter','latex','FontSize',12);
yticklabels({'-1.5\pi','-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi'})
yticks([-1.5*pi -1*pi -0.5*pi 0 0.5*pi 1*pi 1.5*pi]);
grid on
xt = get(gca, 'XTick');
set(gca, 'FontSize', 12);
yt = get(gca, 'YTick');
set(gca, 'FontSize', 12);
hold on

%
[x_q2,y_q2] = ginput(5);
% 

 
% 
% 
   fit = polyfit(x_q2(1:end), log(y_q2),1)
% %

zeta_2 =sqrt( 1/( 1+ q2_wd^2/fit(1)^2 ) )
%zeta_2=0.85;
q2_wn = q2_wd/sqrt(1-zeta_2^2)
A_2 = exp(fit(2))
% 

plot(time_q2,A_2.*exp(-1*zeta_2*q2_wn.*time_q2));

