% F16 pitch damper

clear all
close all
clc

load('12km330kmh_f16_long.mat') % from FindF16Dynamics.m

% Convert state from:
% 1 h
% 2 theta
% 3 V
% 4 alpha
% 5 q
% 6 dt
% 7 de

% To:
% 1 theta
% 2 V
% 3 alpha
% 4 q

A = SS_long_lo.A(2:5,2:5);
B = SS_long_lo.A(2:5,7);
C = SS_long_lo.C(2:5,2:5);
D = SS_long_lo.D(2:5,2);

% pitch rate q in deg/s
C = C(4,:);
D = D(4,:);

long_red = ss(A,B,C,D, 'StateName',SS_long_lo.StateName(2:5), 'InputName', 'elevator','Name','F16 long');

H = minreal(tf(long_red));

figure
step(H)
ylabel('$q$ [deg/s]','interpreter', 'latex')

% positive deflection is elevator down -> gain needs to be negative
% sisotool works only with a positive gain, so use -H! 
sisotool(-H)
%%

K                   = -0.5;
figure
step(K*H)

sys_lo_cl           = K*H/(1+K*H)
% n = [0.4661, 0.2768, 0.358, 0.0]
% [A,B,C,D] = tf2ss(n,d)

hold on

step(sys_lo_cl)
ylabel('$q$ [deg/s]','interpreter', 'latex')




%% Figure for in the slides
figure;
rlocus(-H);
hold on;
plot(real(pole(minreal(H/(1+K*H)))), imag(pole(minreal(H/(1+K*H)))),...
    's','MarkerSize',6,'MarkerEdgeColor','magenta','MarkerFaceColor','magenta')
grid on;
xlim([-1.5 0.1])

%% Washout filter

figure;
bodemag(1-tf(0.1,[1 +0.1]));
grid on;
hold on;
plot([0.1 0.1],[1 -100],'r');
xlim([0.01 100]);
ylim([-20 0]);
text(0.15,-16,'$\omega_c$','interpreter','latex'); 
pbaspect([2 1 1])

% Short period frequency bode
figure;
bodemag(-H)
xlim([0.01 10])
grid on
pbaspect([2 1 1])