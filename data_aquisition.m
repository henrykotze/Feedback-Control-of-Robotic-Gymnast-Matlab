%% Data Acquisition

% Data structure within .csv file:
% time,q1,q2,tau

% file name for q1
file_name_q1 = 'C:\Users\Henry\Desktop\Skripsie\Feedback-Control-of-Robotic-Gymnast-MCU\q1_response6.csv';

% file name for q2
file_name_q2 = 'C:\Users\Henry\Desktop\Skripsie\Feedback-Control-of-Robotic-Gymnast-MCU\q2_response4.csv';

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
figure(1);
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

% filter to use: moving,lowess, loess, sgolay,rlowess,rloess

abs_fft_q1 = abs(fft(q1));
abs_fft_q2 = abs(fft(q2));











figure(3);
plot(abs_fft_q1(1:100),'-*r', 'LineWidth',2)
grid on
hold on
plot(abs_fft_q2(1:100),'-*b', 'LineWidth',2)
title('Frequency Content of Initial Condition System Responses','Interpreter','latex','FontSize',12)
ylabel('Magnitude ','Interpreter','latex','FontSize',12);
xlabel('Frequency [Hz]','Interpreter','latex','FontSize',12);
legend({'System Response with $\phi = 0$ rad','System Response with  $\theta = 0$'},'Interpreter','latex','FontSize',12)



