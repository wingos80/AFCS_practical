%% Trim F16 at 10k ft and 700 ft/s
load('F16−10kft700fts_long.mat')
% To find out the names of the states, execute SS long lo in the
% command window.
A = SS_long_lo.A;
B = SS_long_lo.B;
C = SS_long_lo.C;
D = SS_long_lo.D;
% obtain transfer functions from all inputs to all outputs
tfs = tf(SS_long_lo);
qtf = tfs(5,2); % select elevator to q transfer function
% Use the root locus to find a gain for q. The short period is already well
% damped, but if we want the outer loops to have a fast response, it helps
% to have more damping in the inner loop.
rltool(qtf)
% Choose Kq = −0.183 deg/(deg/s)
Kq = -0.183;

%% Pitch attitude hold mode
% In this case, since we want to make a cascade of a pitch rate and a pitch
% angle controller, we can find the closed loop transfer function of the
% pitch rate controller by:
qcl = minreal(Kq*qtf/(1+Kq*qtf));
% For the pitch angle controller we can then find a gain with:
integrator = tf(1,[1 0]);
Kt = 2; % deg/s / deg
theta_ol = Kt*qcl*integrator;
figure;
rlocus(theta_ol)
figure;
step(feedback(theta_ol,1))
grid on
%%
% To go back to a state space system:
Aaug = A - B(:,2)*C(5,:)*Kq - B(:,2)*C(2,:)*Kq*Kt;
qthetafb = ss(Aaug, B, C, D, 'StateName', SS_long_lo.StateName, 'InputName', SS_long_lo.InputName);
tfs = tf(qthetafb);
% Obtain the transfer function from elevator to altitude. If we multiply
% this with the gains of the pitchrate and pitch angle controllers, we get
% the transfer function from pitch angle reference to altitude.
htf = tfs(1,2)*Kq*Kt;
% rltool(htf)
s = tf('s');
Kh = 0.01+0.0003/s; % integrator to get rid of the bias
h_ol = Kh*htf;
hcl = minreal(h_ol/(1+h_ol));
figure
rlocus(h_ol)
figure
step(50*hcl)
%% Lead compensator
Kh = 0.01+0.0003/s; % integrator to get rid of the bias
h_ol_compensator = Kh*htf*(s+0.4)/(s+2);
% Obtain the root locus including the compensator and PI controller.
% A gain of 20
figure
rlocus(h_ol_compensator)
% Multiply with the PI controller with the gain found above
Kh_new = 20*(0.01+0.0003/s);
h_ol_compensator = Kh_new*htf*(s+0.4)/(s+2);
hcl_compensator = (h_ol_compensator/(1+h_ol_compensator));
% 50m step response
figure
step(50*hcl)
hold on
step(50*hcl_compensator)
legend('no compensator','with compensator')
ylabel('Altitude [ft]')