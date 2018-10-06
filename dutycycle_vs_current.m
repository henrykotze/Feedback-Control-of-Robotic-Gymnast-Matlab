%% Relationship between PWM signal duty cycle and current through motor
% 
close all
clear all
% include .csv file containing information
torque_constant = 19.1e-3; %[Nm]
gearbox_ratio = 14 ;
motor_driver_ratio =375;
Resistor = 150; % [Ohm]

duty_cycle = 20:1:43;
voltage = [42 58 69 88 110 120 140 154 180 203 222 240 265 304 320 355 370 400 422 470 490 533 547 578 ]./(Resistor).*motor_driver_ratio.*torque_constant.*gearbox_ratio;


% [to do]
% fit linear line, exponential line to data
[fit] = polyfit(duty_cycle,voltage,1)

approx_y = fit(1).*duty_cycle + fit(2);
R = corrcoef(voltage,approx_y)
figure(1)

plot(duty_cycle,voltage,'or','LineWidth',2);
hold on
title('Relationship between Duty-Cycle of PWM Signal \& Torque','Interpreter','latex','FontSize',12)
plot(duty_cycle,approx_y,'-b','LineWidth',2)
ylabel('Torque Delivered by Motor [mNm]','Interpreter','latex','FontSize',12);
xlabel('Duty Cycle of PWM signal [\%]','Interpreter','latex','FontSize',12);
legend({'Experimental Data','Linear Fit: $y = 15.8283-312.5268$'},'Interpreter','latex','FontSize',12)
grid on
xt = get(gca, 'XTick');
set(gca, 'FontSize', 12);
yt = get(gca, 'YTick');
set(gca, 'FontSize', 12);


print -depsc2 dutycycle_vs_current.eps
print -djpeg dutycycle_vs_current.jpg