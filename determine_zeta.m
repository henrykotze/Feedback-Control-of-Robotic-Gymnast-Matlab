%% Determine zeta for response

%% Natural frequencies of response
% Requires natural frequency determine in data_aquisition.m
q1_wn = 3.975275*2*pi;
q2_wn = 1.2*2*pi;


% file name for q1
file_name_q1 = 'C:\Users\Henry\Desktop\Skripsie\Feedback-Control-of-Robotic-Gymnast-MCU\q1_response1.csv';
% file name for q2
file_name_q2 = 'C:\Users\Henry\Desktop\Skripsie\Feedback-Control-of-Robotic-Gymnast-MCU\q2_response2.csv';

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
q1 = (system_info_q1(1:end,2)-1570).*0.00153;

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

% for beginning part
[x_q11,y_q11] = ginput(10);
fit = polyfit(x_q11(1:end), log(-1.*y_q11),1)
zeta_11 = fit(1)./(-q1_wn)
A_11 = exp(fit(2))
plot(time_q1, A_11.*exp(-1*zeta_11*q1_wn*time_q1),'-b','LineWidth',2);
% for end part
[x_q12,y_q12] = ginput(5);
fit = polyfit(x_q12(1:end), log(-1.*y_q12),1)
zeta_12 = fit(1)./(-q1_wn)
A_12 = exp(fit(2))
plot( time_q1(3000:end,1), A_12.*exp(-1*zeta_12*q1_wn*time_q1(3000:end,1)),'-g','LineWidth',2);





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
% [x_q2,y_q2] = ginput(3);
% 
% 
% 
% 
% fit = polyfit(x_q2(1:end), log(y_q2),1)
% 
% zeta_2 = fit(1)./(-q2_wn)
% A_2 = exp(fit(2))
% 
% plot(time_q2, A_1.*exp(-1*zeta_1*q2_wn*time_q2));

