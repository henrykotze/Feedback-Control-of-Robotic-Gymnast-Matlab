%% playground
% script to test things


f = 20;

wn = 2*pi*f;

t = linspace(0,1,1000);

y = sin(wn*t);

plot(abs(fft(y)))