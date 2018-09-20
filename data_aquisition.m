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
print -depsc2 q1_initial_response.eps

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
print -depsc2 q2_initial_response.eps
% filter to use: moving,lowess, loess, sgolay,rlowess,rloess

abs_fft_q1 = abs(fft(q1));
abs_fft_q2 = abs(fft(q2));

Fs = 1000;          % sampling rate
T = 1/Fs;
L_q1 = length(time_q1);
time_q1=time_q1./1000;
Y_q1 = fft(q1);
P2_q1 = abs(Y_q1/L_q1);
P1_q1 = P2_q1(1:L_q1/2+1);
P1_q1(2:end-1) = 2*P1_q1(2:end-1);
f_q1 = Fs*(0:(L_q1/2))/L_q1;

L_q2 = length(time_q2);
time_q2=time_q2./1000;
Y_q2 = fft(q2);
P2_q2 = abs(Y_q2/L_q2);
P1_q2 = P2_q2(1:L_q2/2+1);
P1_q2(2:end-1) = 2*P1_q2(2:end-1);
f_q2 = Fs*(0:(L_q2/2))/L_q2;



fig3 = figure(3);

plot(f_q1(1:80),P1_q1(1:80),'-*r', 'LineWidth',2) 
grid on
hold on
plot(f_q2(1:20),P1_q2(1:20),'-*b', 'LineWidth',2)
title('Frequency Content of Initial Condition System Responses','Interpreter','latex','FontSize',12)
ylabel('Magnitude ','Interpreter','latex','FontSize',12);
xlabel('Frequency [Hz]','Interpreter','latex','FontSize',12);
legend({'System Response with $\phi = 0$ rad','System Response with  $\theta = 0$ rad'},'Interpreter','latex','FontSize',12)
dcm_obj = datacursormode(fig3);
set(dcm_obj,'UpdateFcn',@myupdatefcn,'DisplayStyle','datatip', 'SnapToDataVertex','on','Enable','on');
% set(dcm_obj,'UpdateFcn',@myupdatefcn)

c_info = getCursorInfo(dcm_obj);
c_info = getCursorInfo(dcm_obj);

disp('Click line to display a data tip, then press Return.')
% Wait while the user does this.
pause 
print -depsc2 FFT_system.eps

%% Curve fitting zeta for q1 response
wn = 24.5421;

decay_function = @(zeta,time_q1)-1.61*exp(-zeta*wn*time_q1).*cos(wn*sqrt(1-zeta_lsq^2).*time_q1);

zeta_start = 0.02;

zeta_lsq = lsqcurvefit(decay_function,zeta_start,time_q1./1000,q1)
zeta_lsq = 0.02;
 decay_approx = -1.6*exp(-zeta_lsq*wn*time_q1);
response_approx = -1.61*exp(-zeta_lsq*wn*time_q1).*cos(wn*sqrt(1-zeta_lsq^2).*time_q1);

figure(4)
plot(time_q1,q1,'r');



%%




