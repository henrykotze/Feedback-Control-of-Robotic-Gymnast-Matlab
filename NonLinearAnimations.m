%% Visualize the dynamics of the system
%%
clear all;
close all;
% fetch variables of systems
run('system_variables');
%% time step for simulation
step_size = 0.0001;          % time step
time = 35;                      % time lenght of the simulation
%% initial condition:
theta_start = 0.2;            % [rad]
phi_start = 0.0;              % [rad]
theta_dot_start = 0.0;      % [rad/s]
phi_dot_start = 0.0;       % [rad/s]
state = 4;

%% Simulation constants
Kp = 58;
Kd = 14;
angle_restrict = pi/4;
region_1 = pi/250;
region_2 = pi/8;
encoder_resolution = 1/896;
ADC_resolution = 1/4095;

%%
% Saturation
motor_stall_torque = 6;
%
play_sim = 1;
% run simulations
sim('NonlinearModel_NonlinearController');

%Retrieve Data from simulation
q1 = get(q1_angle,'Data');
q2 = get(q2_angle,'Data');

q2dot = get(q2dot,'Data');
q1dot = get(q1dot,'Data');
q2dotdot = get(q2dotdot,'Data');
q1dotdot =get(q1dotdot,'Data');
tau_state = get(tau_state,'Data');

tau = get(tau,'Data');
% Verifying simulation is correct with energy principle with no friction
close all
T = 0.5.*Ia.*q1dot.^2 + 0.5.*Ib.*(q1dot+q2dot).^2 +0.5.*m2.*(L1^2.*q1dot.^2+l2^2.*(q1dot+q2dot).^2+2*L1*l2.*q1dot.*(q1dot+q2dot).*cos(q2));
V = -m1*g*l1.*cos(q1) - m2*g*L1.*cos(q1) - m2*g*l2.*cos(q1+q2);
L = T+V;
% Langarange Energy Principle
figure(5)
plot(L);
title('Mechanical Energy in the System','Interpreter','latex','FontSize',12);
xlabel('Time -[s]','Interpreter','latex','FontSize',12);
ylabel('Mechanical Energy','Interpreter','latex','FontSize',12);

% Point B: [x,y] positions for elbow
pointB = [L1*sin(q1), -L1*cos(q1)];
% Point C: [x,y] position of tip of lower pendulum
pointC = [L1*sin(q1)+L2*sin(q2+q1), -L1*cos(q1)-L2*cos(q2+q1)];


x_points = [zeros(size(pointB(1:end,1),1),1), pointB(1:end,1), pointC(1:end,1)];
y_points = [zeros(size(pointB(1:end,2),1),1), pointB(1:end,2), pointC(1:end,2)];

timestep  = 1:1:size(pointB(1:end,1));

% \phi angle 
figure(1)
plot(timestep.*step_size,q2, '.', 'LineWidth',1);
title('$\phi$ with respect to time ','Interpreter','latex','FontSize',12)
ylabel('$\phi$ [rad]','Interpreter','latex','FontSize',12);
xlabel('time [s]','Interpreter','latex','FontSize',12);
yticks([-1.5*pi -pi -0.5*pi 0 0.5*pi pi 1.5*pi]);
yticklabels({'-1.5\pi', '-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi'})
grid on

% \theta angle
figure(2)
plot(timestep.*step_size,q1, '.', 'LineWidth',1)
title('$\theta$ with respect to time','Interpreter','latex','FontSize',12)
yticks([-2*pi -pi 0 pi 2*pi]);
ylabel('$\theta$ -[rad]','Interpreter','latex','FontSize',12);
xlabel('time [s]','Interpreter','latex','FontSize',12);
yticklabels({'-1.5\pi','-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi'})
yticks([-1.5*pi -1*pi -0.5*pi 0 0.5*pi 1*pi 1.5*pi]);
grid on

% \tau
figure(3)
plot(timestep.*step_size,tau, '-', 'LineWidth',1);
title('$\tau$ with respect to time','Interpreter','latex','FontSize',12)
xlabel('time [s]','Interpreter','latex','FontSize',12);
ylabel('$\tau$ [Nm]','Interpreter','latex','FontSize',12);
grid on  
axis normal

% \theta and \phi
figure(4)
plot(timestep.*step_size,q1, '-', 'LineWidth',2)
hold on
grid on
plot(timestep.*step_size,q2, '-', 'LineWidth',2);
title('Angular Position of Robotic Gymnast during Swing-up \& Balancing','Interpreter','latex','FontSize',12)
ylabel('$\phi$ and $\theta$ [rad]','Interpreter','latex','FontSize',12);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
legend({'$\theta$','$\phi$'},'Interpreter','latex','FontSize',12)
yticks([-2*pi -1.5*pi -pi -0.5*pi 0 0.5*pi pi 1.5*pi 2*pi]);
yticklabels({'-2\pi', '-1.5\pi', '-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi','2\pi'})
% plot(timestep.*step_size,tau_state,'LineWidth',2);

figure(6)
plot(timestep.*step_size,tau_state,'LineWidth',4);
title('The State of the controllers');
% 

% q1dot and q2dot
figure(7)
plot(timestep.*step_size,q1dot, '-', 'LineWidth',1)
hold on
grid on
plot(timestep.*step_size,q2dot, '-', 'LineWidth',1);
title('Angular Velocity of Double Pendulum during Swing-up','Interpreter','latex','FontSize',12)
ylabel('$\dot{\phi}$ and $\dot{\theta}$ [rad/s]','Interpreter','latex','FontSize',12);
xlabel('time [s]','Interpreter','latex','FontSize',12);
legend({'$\dot{\theta}$','$\dot{\phi}$'},'Interpreter','latex','FontSize',12)
yticks([-2*pi -1.5*pi -pi -0.5*pi 0 0.5*pi pi 1.5*pi 2*pi]);
yticklabels({'-2\pi', '-1.5\pi', '-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi','2\pi'})

figure(8)
plot(timestep.*step_size,q1, '-', 'LineWidth',2)
hold on
grid on
plot(timestep.*step_size,q2, '-', 'LineWidth',2);
plot(timestep.*step_size,q1dot, '-', 'LineWidth',2);
plot(timestep.*step_size,q2dot, '-', 'LineWidth',2);
title('Angular Velocity \& Angular Position of Pendulum during Swing-up','Interpreter','latex','FontSize',12)
ylabel('$\phi$ $\theta$ and $\dot{\phi}$  $\dot{\theta}$ [rad] [rad/s]','Interpreter','latex','FontSize',12);
xlabel('time [s]','Interpreter','latex','FontSize',12);
legend({'$\phi$','$\theta$','$\dot{\theta}$','$\dot{\phi}$'},'Interpreter','latex','FontSize',12)
yticks([-2*pi -1.5*pi -pi -0.5*pi 0 0.5*pi pi 1.5*pi 2*pi]);
yticklabels({'-2\pi', '-1.5\pi', '-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi','2\pi'})

%q1dot vs q1
figure(9)
plot(q1dot,q1, '-', 'LineWidth',1)
title('Angular Position vs Angular Velocity of Non-Actuated Pendulum during Swing-up \& Balancing','Interpreter','latex','FontSize',12)
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',12);
xlabel('$\dot{\theta}$ [rad/s]','Interpreter','latex','FontSize',12);
yticks([-3*pi -2.5*pi -2*pi -1.5*pi -pi -0.5*pi 0 0.5*pi pi 1.5*pi 2*pi 2.5*pi 3*pi]);
yticklabels({'-2\pi', '-1.5\pi', '-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi','2\pi'})
xticks([-4.5*pi -4*pi -3.5*pi -3*pi -2.5*pi -2*pi -1.5*pi -pi -0.5*pi 0 0.5*pi pi 1.5*pi 2*pi 2.5*pi 3*pi 3.5*pi 4*pi 4.5*pi]);
xticklabels({'-4.5\pi','-4\pi','-3.5\pi','-3\pi', '-2.5\pi','-2\pi', '-1.5\pi', '-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi','2\pi','2.5\pi','3\pi','3.5\pi','4\pi','4.5\pi}'})
grid on



% q2dot vs q2
figure(10)
plot(q2dot,q2, '-', 'LineWidth',1)

% tau with q2;
figure(11);
plot(tau);
hold on
plot(q2);
plot(q2dot);

error = pi/6*atan(q1dot)-q2;
q2_ref  = pi/6*atan(q1dot);

% Figure 12
figure(12);
plot(timestep.*step_size,q1, '-', 'LineWidth',2)
hold on
grid on
plot(timestep.*step_size,q2, '-', 'LineWidth',2);
plot(timestep.*step_size,q1dot, '-', 'LineWidth',2)
plot(timestep.*step_size,q2dot, '-', 'LineWidth',2);
plot(timestep.*step_size,error, '-', 'LineWidth',2);
plot(timestep.*step_size,q2_ref, '-', 'LineWidth',2);
legend('q2','q1','q1dot','q2dot','error: atan(q1dot) - q2', 'q2_{ref}')


% Figure 13
% figure(13)
% Fs = 1/step_size;          % sampling rate
% T = 1/Fs;
% L_q1 = length(timestep.*step_size);         % number of samples taken
% time_q1=timestep.*step_size;      
% Y_q1 = fft(q1);                     % FFT of response
% P2_q1 = abs(Y_q1);              % absolute value of FFT 
% P1_q1 = P2_q1(1:L_q1/2+1);
% P1_q1(2:end-1) = 2*P1_q1(2:end-1);
% f_q1 = Fs*(0:(L_q1/2))/L_q1;
% 
% L_q2 = length(timestep.*step_size);
% time_q2=timestep.*step_size;
% Y_q2 = fft(q2);
% P2_q2 = abs(Y_q2);
% P1_q2 = P2_q2(1:L_q2/2+1);
% P1_q2(2:end-1) = 2*P1_q2(2:end-1);
% f_q2 = Fs*(0:(L_q2/2))/L_q2;
% plot(f_q1(1:80),P1_q1(1:80),'-*r', 'LineWidth',2) 
% grid on
% hold on
% plot(f_q2(1:80),P1_q2(1:80),'-*b', 'LineWidth',2)
% title('Frequency Content of Initial Condition System Responses','Interpreter','latex','FontSize',12)
% ylabel('Magnitude ','Interpreter','latex','FontSize',12);
% xlabel('Frequency [Hz]','Interpreter','latex','FontSize',12);
% legend({'System Response with $\phi = 0$ rad','System Response with  $\theta = 0$ rad'},'Interpreter','latex','FontSize',12)




% figure(5)
% plot(q2,q1);
% tit = title('$\dot{\theta} vs \theta$');
% set(tit,'Interpreter','latex');
% figure(6);
% plot(q2dot,q1dot);
%% NEW Version
if play_sim == 1

    prompt = 'Play Simulation: [yes == 1 / no == 0]?:';
    reply = input(prompt);
    if reply == 1

        prompt = 'Starting time: ';
        time_start = input(prompt);

        P0 = [0 0];

        % axis(goa,'equal');
        figure(100)
        axis([-L1-L2, L1+L2, -L1-L2, L1+L2]);
        %axis vis3d
        ax = gca;
        ax.GridAlpha = 1;
        grid on
        %% Graph Settings


        pendulum = animatedline('Marker','o');

        L1_dx = L1;
        L2_dx = L2;
        text(L1/2-0.05,L1,'Non-Actuated Pendulum','Color','Red','FontSize',12);
        text(L1/2,L1/2,'Actuated Pendulum','Color','Black','FontSize',12);
        num_frames = time/step_size;
        clear F;
          F(num_frames) = struct('cdata',[],'colormap',[]);

        time_steps = 1:length(tout);
    % 
%           pause(0.0001);
           v = VideoWriter('swing-up&balancing.mp4');
          v.Quality = 100;
          v.FrameRate = 100;
         open(v)

        for step = (time_start/step_size):10:length(tout)
            P1 = [L1_dx.*sin(q1(step)) -L1_dx*cos(q1(step))];
            P2 = [L1_dx.*sin(q1(step))+L2_dx.*sin(q1(step)+q2(step)), -L1_dx.*cos(q1(step))-L2_dx.*cos(q1(step)+q2(step)) ];
%             addpoints(pendulum, [P1(1),P2(2)], [P1(2),P2(2)] );

            upper_pendulum = line( [P0(1) P1(1)], [ P0(2), P1(2) ],'Color','r','LineWidth',2);
            lower_pendulum = line( [P1(1) P2(1)], [ P1(2), P2(2) ],'Color','k','LineWidth',2);
            drawnow
            F(step) = getframe(gcf);
            pause(0.001);
               frame = getframe;
           writeVideo(v,frame);
            delete(upper_pendulum);
            delete(lower_pendulum);

        end
    end
end

if reply == 1
    close(v);
end
disp('Simulation Finished');


