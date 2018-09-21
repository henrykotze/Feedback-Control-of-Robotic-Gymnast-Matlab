%% Relationship between PWM signal duty cycle and current through motor
% 

% include .csv file containing information

duty_cycle = 20:1:34;
voltage = [42 58 69 88 110 120 140 154 180 203 222 240 265 304 320]./(150).*375

% [to do]
% fit linear line, exponential line to data
[fit] = polyfit(duty_cycle,voltage,1);

approx_y = fit(1).*duty_cycle + fit(2);
R = corrcoef(voltage,approx_y)
figure(1)

plot(duty_cycle,voltage,'or','LineWidth',2);
hold on
title('Relationship between Duty-Cycle of PWM Signal \& Current','Interpreter','latex','FontSize',12)
plot(duty_cycle,approx_y,'-b','LineWidth',2)
ylabel('Current through Motor [mA]','Interpreter','latex','FontSize',12);
xlabel('Duty Cycle of PWM signal [\%]','Interpreter','latex','FontSize',12);
legend({'Experimental Data','Linear Fit: $y = 19x -326$'},'Interpreter','latex','FontSize',12)
grid on
xt = get(gca, 'XTick');
set(gca, 'FontSize', 12);
yt = get(gca, 'YTick');
set(gca, 'FontSize', 12);


print -depsc2 dutycycle_vs_current.eps