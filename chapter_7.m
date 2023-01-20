%================================================
% Author: W. Chan
% Date:   18 Jan 2023
%================================================
clear

global fi_flag_Simulink

%% Trim aircraft to desired altitude and velocity
%%
% lastname B., last number 300
% sergio_report's nums
% altitude = 10000;
% velocity = 350;
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

sys_sp = ss(long_A_sp, long_B_sp, long_C_sp, long_D_sp);


%% Construct the 4 state longitudinal model
long_states_4 = [5, 7, 8, 11]; % [theta, Vt, alpha, q]
long_inputs_4 = 14;             % [elevator_cmd]

long_A_4 = SS_lo.A(long_states_4, long_states_4);
long_B_4 = SS_lo.A(long_states_4, long_inputs_4);
long_C_4 = SS_lo.C(long_states_4, long_states_4);
long_D_4 = SS_lo.C(long_states_4, long_inputs_4);


sys_4 = ss(long_A_4, long_B_4, long_C_4, long_D_4);
%% Plotting the step response of sp and 4 states models

h = figure(1);

t = 0:0.01:10;
y_sp = step(sys_sp, t);
y1 = y_sp(:,2);
plot(t,y1, 'DisplayName', '2 states', 'LineWidth', 2);hold on 

y_4 = step(sys_4, t);
y2 = y_4(:,4);
plot(t,y2, 'DisplayName', '4 states', 'LineWidth', 2);

grid
legend
xlabel('Time [s]')
ylabel('q [deg/s]')

set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'figs/4states_vs_2states_short','-dpdf','-r0')

h = figure(2);

t = 0:0.01:100;
y_sp = step(sys_sp, t);
y1 = y_sp(:,2);
plot(t,y1, 'DisplayName', '2 states', 'LineWidth', 2);hold on 

y_4 = step(sys_4, t);
y2 = y_4(:,4);
plot(t,y2, 'DisplayName', '4 states', 'LineWidth', 2);
grid
legend
xlabel('Time [s]')
ylabel('q [deg/s]')

set(h,'Units','Inches'); pos = get(h,'Position'); set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'figs/4states_vs_2states_long','-dpdf','-r0')

%% finding the gains which provide the prescribed omega and zeta
% incorrect roots, aka sergios reports roots
% p1 = -1.62 + 2.77*1i;
% p2 = -1.62 - 2.77*1i;

% correct roots
p1 = -1.3716 + 2.3757*1i;
p2 = -1.3716 - 2.3757*1i;
ps = [p1, p2];

K = place(long_A_sp, long_B_sp, ps);
Acl = long_A_sp - long_B_sp*K;
syscl = ss(Acl, long_B_sp, long_C_sp, long_D_sp);

%% finding elevator deflection in case of worst case wind gust
wind = 4.572;
alpha_induced = atan(wind/(velocity*0.3048));

d_el = K(1)*alpha_induced;

%% finding our elevator to pitch rate transfer function
tf_all_new = tf(syscl);
tf_q_new = tf_all_new(2);
%% Defining FF transfer function
s = tf('s');

% incorrect ff
% num_FF = [0.714, 1];
% den_FF = [5.676, 1];

% correct ff
num_FF = [0.4861, 1];
den_FF = [5.676, 1];

% sergio_report's nums
% num_FF = [0.4166, 1];
% den_FF = [1.98, 1];

FF = tf(num_FF, den_FF);

%% Defining KSS
% incorrect k_ss
% K_ss = -40.23;

% correct k_ss
K_ss = -29.4;

% sergio_report's nums
% K_ss = -5.3887;

%% Rectangular input
dt = 0.01;
t = 0:dt:40;
u = zeros(1, 4001);
u(1, 1:2000) = 1;


sys_new = FF*tf_q_new*K_ss;

tf_all_old = tf(sys_sp);
tf_q_old = tf_all_old(2);
sys_old = -tf_q_old;

response_new = lsim(sys_new, u, t);
response_old = lsim(sys_old, u, t);

h = figure(3);
plot(t, u, 'DisplayName', 'Input', 'LineWidth', 2);
hold on
plot(t, response_new, 'DisplayName', 'Design Point', 'LineWidth', 2);
hold on
plot(t, response_old, 'DisplayName', 'Initial Point', 'LineWidth', 2);
hold on
grid
legend
xlabel('Time [s]')
ylabel('q [deg/s]')

set(h,'Units','Inches'); pos = get(h,'Position'); set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'figs/q_reponse_compare','-dpdf','-r0')

h = figure(4);
theta_u = Produce_theta(u, dt);
plot(t, theta_u, 'DisplayName', 'Input', 'LineWidth', 2)
hold on
theta_new = Produce_theta(response_new, dt);
plot(t, theta_new, 'DisplayName', 'Design Point', 'LineWidth', 2)
hold on
theta_old = Produce_theta(response_old, dt);
plot(t, theta_old, 'DisplayName', 'Initial Point', 'LineWidth', 2)
hold on

grid
legend
xlabel('Time [s]')
ylabel('\theta [degs]')


set(h,'Units','Inches'); pos = get(h,'Position'); set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'figs/theta_reponse_compare','-dpdf','-r0')
%% CAP and Gibson criteria
omega_n_sp = 0.03*0.3048*velocity;
inv_over_T_theta = 0.75*omega_n_sp;
T_theta = 1/inv_over_T_theta;
zeta_sp = 0.5;

new_CAP = get_CAP(omega_n_sp, velocity, 9.80665, T_theta);
new_DB = get_DB(T_theta, zeta_sp, omega_n_sp);
% new_qmqs = 1.82977;
new_qmqs = max(response_new);

old_CAP = 0.395;
old_DB = 4.86711;
old_qmqs = 3.67;

% plotting gibsons criteria
h = figure(5);
title('Original comparison of Gibsons Criteria')
a = area([0 0 0.08 0.3],[0 3 3 0],'FaceColor','g', 'DisplayName', 'Satisfactory Region'); hold on
a.FaceAlpha = 0.5;
plot(new_DB, new_qmqs, 'x', 'Color', 'r', 'DisplayName', 'Design Point', 'LineWidth', 2);hold on 
plot(old_DB, old_qmqs, 'x', 'Color', 'b', 'DisplayName', 'Initial Point', 'LineWidth', 2);hold on 
xlabel('DB/q_{ss} [s]');
ylabel('q_m/q_s [-]');
xlim([-0.2 5.2]);
ylim([0 5.2]);
% set(gca, 'XScale', 'log')
grid on
legend

set(h,'Units','Inches'); pos = get(h,'Position'); set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'figs/gibs_compare','-dpdf','-r0')


%% Gain Scheduling: finding gains for a bunch of velocites
upper_velocity = 600;
t = 0:dt:10;
u = zeros(1, 1001);
u(1, 1:500) = 1;

figure(6)
plot(t, u, 'DisplayName', 'Input', 'LineWidth', 2);hold on

grid; legend;
xlabel('Time [s]'); ylabel('q [deg/s]');


figure(7)
theta_u = Produce_theta(u, dt);
plot(t, theta_u, 'DisplayName', 'Input', 'LineWidth', 2);hold on

grid; legend;
xlabel('Time [s]'); ylabel('\theta [degs]');


figure(8);
title('Gain Scheduling comparison of Gibsons Criteria')
a = area([0 0 0.08 0.3],[0 3 3 0],'FaceColor','g', 'DisplayName', 'Satisfactory Region'); hold on
a.FaceAlpha = 0.5;
xlabel('DB/q_{ss} [s]');
ylabel('q_m/q_s [-]');
xlim([-0.1 0.4]);
ylim([0 3.8]);
grid on
legend

altitude = 5000;
for v = velocity:100:upper_velocity
    disp(['Gain Scheduling for velo = ' num2str(v) ' ft/s'])
    %% Find trim for lofi model at desired altitude and velocity
    %%
    disp('Trimming Low Fidelity Model:');
    fi_flag_Simulink = 0;
    [trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, v, altitude, FC_flag);
    
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
       
    omega = 0.03*0.3048*v;
    inv_T = 0.75*omega;
    T = 1/inv_T;
    zeta = 0.5;
    
    quadratic = [1 omega omega^2];
    poles = roots(quadratic);
    poles = [ poles(1) poles(2)];
    
    K = place(long_A_sp, long_B_sp, poles)
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
    
    FF = tf(num_FF, den_FF)
    
    sys_new = FF*tf_q_new*K_ss;
    response_new = lsim(sys_new, u, t);
    
    string = join([num2str(v), ' ft/s']);
    
    figure(6)
    plot(t, response_new, 'DisplayName', string, 'LineWidth', 2);hold on
    
    
    figure(7)
    theta_new = Produce_theta(response_new, dt);
    plot(t, theta_new, 'DisplayName', string, 'LineWidth', 2);hold on
    
    new_cap = get_CAP(omega, v, 9.80665, T);
    new_db = get_DB(T, zeta, omega);
    
    %% plotting gibsons stuff
    new_qmqs = max(response_new);
    
    figure(8);
    plot(new_db, new_qmqs, 'x', 'DisplayName', string, 'LineWidth', 2);hold on 
   
    disp(['For v ' num2str(v) ' ft/s: CAP = ' num2str(new_cap) ', DB = ' num2str(new_db) ', qm/qs = ' num2str(new_qmqs) ', K_ss = ' num2str(K_ss)])
    K
    FF
end

h = figure(6);
set(h,'Units','Inches'); pos = get(h,'Position'); set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'figs/scheduling_qs','-dpdf','-r0')

h = figure(7);
set(h,'Units','Inches'); pos = get(h,'Position'); set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'figs/scheduling_thetas','-dpdf','-r0')


h = figure(8);
set(h,'Units','Inches'); pos = get(h,'Position'); set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'figs/scheduling_gibss','-dpdf','-r0')



function theta = Produce_theta(x, dt)
    theta = zeros(1, length(x));
    for i = 1:length(x)
        theta(i) = Integrate_list(x(1:i),dt);
    end
end

function integrand = Integrate_list(x, dt)
    integrand = 0;
    for i = 1:(length(x)-1)
        integrand = integrand + dt*(x(i)+x(i+1))/2;
    end
end

function CAP = get_CAP(omega, vel, g, T)
    CAP = T*g*omega*omega/(vel*0.3048);
end

function DB = get_DB(T, zeta, omega)
    DB = T - 2*zeta/omega;
end
