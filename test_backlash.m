%% Verifying the working of the backlash functions
close all
angle = linspace(pi/2 - pi/4, pi/2+pi/4,100);



sim('testing_backlash');

figure(1)
plot(tau_output,'LineWidth',2)
hold on 
plot(q2_feed,'r','LineWidth',2)
plot(q2dot_feed,'b', 'LineWidth',2)
plot(backlashActive,'LineWidth',2)
plot(q2hold,'LineWidth',2)
grid on
legend('tau output', 'q2','q2dot','backlashActive','q2hold');
