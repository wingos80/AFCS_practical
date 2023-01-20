clear

global fi_flag_Simulink

altitude = 30000;
velocity = 300;


disp('At what flight condition would you like to trim the F-16?');
disp('1.  Steady Wings-Level Flight.');
disp('2.  Steady Turning Flight.');
disp('3.  Steady Pull-Up Flight.');
disp('4.  Steady Roll Flight.');
FC_flag = 1;

disp('We are working with:');
disp(FC_flag)

x_a = 5.9;
g_d = 32.17; %gravitational acceleration in ft per second squared

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for lofi model at desired altitude and velocity
%%
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

%% Find the state space model for the lofi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);

SS_lo = linearize('accelerometer_LIN_F16Block');

%%  Construct the short period reduced model from the model without actuator dynamics
long_states_sp = [8, 11];             % [alpha, q]
long_inputs_sp = 14;                 % [elevator_cmd]

long_A_sp = SS_lo.A(long_states_sp,long_states_sp);
long_B_sp = SS_lo.A(long_states_sp,long_inputs_sp);
long_C_sp = SS_lo.C(long_states_sp,long_states_sp);
long_D_sp = SS_lo.C(long_states_sp,long_inputs_sp);

% sys_sp = ss(long_A_sp, long_B_sp, long_C_sp, long_D_sp);
   
omega = 0.03*0.3048*velocity;
inv_T = 0.75*omega;
T = 1/inv_T;
zeta = 0.5;

quadratic = [1 omega omega^2];
poles = roots(quadratic);
poles = [ poles(1) poles(2)];

K = place(long_A_sp, long_B_sp, poles);
Acl = long_A_sp - long_B_sp*K;
syscl = ss(Acl, long_B_sp, long_C_sp, long_D_sp);

%% finding our elevator to pitch rate transfer function
tf_all_new = tf(syscl);

tf_q_new = tf_all_new(2);
numerator = tf_q_new.numerator{1,1};
denominator = tf_q_new.denominator{1,1};

T_broken = numerator(2)/numerator(3);
K_ss = denominator(3)/numerator(3);
%% Defining FF transfer function
num_FF = [T, 1];
den_FF = [T_broken, 1];

FF = tf(num_FF, den_FF);

sys_new = FF*tf_q_new*K_ss;


%% playipng with fft!!
dt = 0.01;
upper = 10;
t = 0:dt:upper;
u = zeros(1, upper/dt+1);
u(1, 1:0.1/dt) = 5;

figure;
q_response = lsim(sys_new, u, t);
% q_response = step(sys_new, t);

cut_off = dt/dt;
cut_off2 = 4/dt+1;
q_plot = q_response(cut_off:cut_off2);
t_plot = t(cut_off:cut_off2);
u_plot = u(cut_off:cut_off2);
plot(t_plot, u_plot, 'DisplayName', 'Input', 'LineWidth', 2);hold on
plot(t_plot, q_plot, 'DisplayName', 'q response', 'LineWidth', 2);hold on

grid; legend;
xlabel('Time [s]'); ylabel('q [deg/s]');

figure
Fs = 1/dt;            % Sampling frequency                    
T = upper-cut_off*dt;             % Sampling period       
L = length(q_plot);             % Length of signal
X = q_plot;
Y = fft(X);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title("Single-Sided Amplitude Spectrum of X(t)")
xlabel("f (Hz)")
ylabel("|P1(f)|")
