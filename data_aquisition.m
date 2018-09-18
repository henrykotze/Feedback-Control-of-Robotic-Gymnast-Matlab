%% Data Acquisition

% Data structure within .csv file:
% time,q1,q2,tau

% file name:
file_name = 'C:\Users\Henry\Desktop\Skripsie\Feedback-Control-of-Robotic-Gymnast-MCU\test2.csv';

% time increments
time_steps = 1e-3;
system_info = csvread(file_name,2);

time = system_info(1:end,1);
q1 = (system_info(1:end,2)-1570).*0.00153;
q2 = system_info(1:end,3).*0.01570796326/2;
tau = system_info(1:end,4);

% three point central difference
q1dot = cent_diff_n(q1,time_steps,3);
q2dot = cent_diff_n(q2,time_steps,3);
q1dotdot = cent_diff_n(q1dot,time_steps,3);
q2dotdot = cent_diff_n(q2dot,time_steps,3);

%% Data Cleaning
% Removing points outside boundary:
i = find(q1(:,1) > 2*pi);
j = find(q1(:,1) < -2*pi);
q1(i) = 0;
q1(j) = 0;

% Removing points outside boundary:
i = find(q2(:,1) > 2*pi);
j = find(q2(:,1) < -2*pi);
q2(i) = 0;
q2(j) = 0;


figure(1);
plot(time,q1,'.');
title('$\theta$ with respect to time','Interpreter','latex','FontSize',12)
yticks([-2*pi -pi 0 pi 2*pi]);
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',12);
xlabel('time [ms]','Interpreter','latex','FontSize',12);
yticklabels({'-1.5\pi','-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi'})
yticks([-1.5*pi -1*pi -0.5*pi 0 0.5*pi 1*pi 1.5*pi]);
grid on

% Using Logarithmic decrement to determine the damping and natural
% frequency
[x,y] = ginput(7);

%determine the average difference between points
del_time = ( ( x(2:end,1)'-x(1:end-1,1))/ 6 )./1000;

del = 1/6*log( x(1)'/x(6)' )

damping_ratio_q1 = del/sqrt(4*pi^2+del^2)


figure(2);

plot(time,q2,'.');
title('$\phi$ with respect to time ','Interpreter','latex','FontSize',12)
ylabel('$\phi$ [rad]','Interpreter','latex','FontSize',12);
xlabel('time [ms]','Interpreter','latex','FontSize',12);
yticks([-1.5*pi -pi -0.5*pi 0 0.5*pi pi 1.5*pi]);
yticklabels({'-1.5\pi', '-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi'})
grid on


[x,y] = ginput(3);