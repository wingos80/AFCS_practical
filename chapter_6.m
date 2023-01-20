%================================================
% Author: W. Chan
% Date:   18 Jan 2023
%================================================
clear

global fi_flag_Simulink

%% Trim aircraft to desired altitude and velocity
%%
% lastname B., last number 300
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

lat_states = [4, 9, 10, 12]; % phi, beta, p, r
lat_inputs = [15, 16]; % aileron, rudder

%% plotting phugoid eigenmotion time response
figure(1)
phugoid_states = [5, 7, 8, 11];            % [theta, Vt, alpha, q]
phugoid_inputs = 14;                 % [elevator_cmd]
phu_A = SS_lo.A(phugoid_states, phugoid_states);
phu_B = SS_lo.A(phugoid_states, phugoid_inputs);
phu_C = SS_lo.C(phugoid_states, phugoid_states); 
phu_D = SS_lo.C(phugoid_states, phugoid_inputs); 

phu_sys = ss(phu_A, phu_B, phu_C, phu_D);

t1 = 0:0.01:100;
opt = stepDataOptions('StepAmplitude',-1);
y_phu = step(phu_sys,t1);

y_t_phu = y_phu(:, 1);
plot(t1,y_t_phu, 'DisplayName', 'Elevator impulse response', 'LineWidth', 2)
grid
legend
xlabel('Time [s]')
ylabel('Theta [deg]')
title('Phugoid')

%% plotting short-period eigenmotion time response
figure(2)
short_states = [8, 11];            % [alpha, q]
short_inputs = 14;                 % [elevator_cmd]

sp_A = SS_lo.A(short_states, short_states);
sp_B = SS_lo.A(short_states, short_inputs);
sp_C = SS_lo.C(short_states, short_states); 
sp_D = SS_lo.C(short_states, short_inputs); 

sp_sys = ss(sp_A, sp_B, sp_C, sp_D);

t2 = 0:0.01:12;
opt = stepDataOptions('StepAmplitude',-1);
y_sp = step(sp_sys,t2);

y_q_sp = y_sp(:, 2);
plot(t2,y_q_sp, 'DisplayName', 'Elevator impulse response', 'LineWidth', 2)
grid
legend
xlabel('Time [s]')
ylabel('Theta [deg]')
title('Short Period')

%% Plotting Dutch roll
figure(3)
dr_states = [4, 9, 10, 12]; % phi, beta, p, r
dr_inputs = [15, 16]; % aileron, rudder

lat_A = SS_lo.A(dr_states, dr_states);
lat_B = SS_lo.A(dr_states, dr_inputs);
lat_C = SS_lo.C(dr_states, dr_states);
lat_D = SS_lo.C(dr_states, dr_inputs);

lat_sys = ss(lat_A, lat_B, lat_C, lat_D);

t = 0:0.01:20;
y_all_lat = impulse(lat_sys,t);

y_roll = y_all_lat(:, 1);

plot(t,y_roll, 'DisplayName', 'Rudder impulse response', 'LineWidth', 2)
grid
legend
xlabel('Time [s]')
ylabel('Roll [deg]')
title('Dutch roll')

long_states = [5, 7, 8, 11];            % [theta, Vt, alpha, q]


%% Plotting aperiodic roll
figure(4)
dr_states = [4, 10]; % phi, p
dr_inputs = [15]; % aileron

lat_A = SS_lo.A(dr_states, dr_states);
lat_B = SS_lo.A(dr_states, dr_inputs);
lat_C = SS_lo.C(dr_states, dr_states);
lat_D = SS_lo.C(dr_states, dr_inputs);

lat_sys = ss(lat_A, lat_B, lat_C, lat_D);

t = 0:0.01:20;
y_all_lat = impulse(lat_sys,t);

y_roll = y_all_lat(:, 1);

plot(t,y_roll, 'DisplayName', 'Aileron impulse response', 'LineWidth', 2)
grid
legend
xlabel('Time [s]')
ylabel('Roll [deg]')
title('Aperiodic roll')

long_states = [5, 7, 8, 11];            % [theta, Vt, alpha, q]

%% Plotting Spiral
figure(5)
dr_states = [4, 9, 10, 12]; % phi, beta, p, r
dr_inputs = [15]; % aileron

lat_A = SS_lo.A(dr_states, dr_states);
lat_B = SS_lo.A(dr_states, dr_inputs);
lat_C = SS_lo.C(dr_states, dr_states);
lat_D = SS_lo.C(dr_states, dr_inputs);

lat_sys = ss(lat_A, lat_B, lat_C, lat_D);

t = 0:0.01:20;
y_all_lat = step(lat_sys,t);

y_roll = y_all_lat(:, 1);

plot(t,y_roll, 'DisplayName', 'Aileron step response', 'LineWidth', 2)
grid
legend
xlabel('Time [s]')
ylabel('Roll [deg]')
title('Spiral')

long_states = [5, 7, 8, 11];            % [theta, Vt, alpha, q]