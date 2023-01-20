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
FC_flag = 1;

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;           % AOA, degrees
rudder = -0.01;         % rudder angle, degrees
aileron = 0.01;         % aileron, degrees


disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude, FC_flag);

x_a = 5.9;
x_a_list = [5.8, 5.82, 5.84, 5.86, 5.88, 5.9];
g_d = 32.17; %gravitational acceleration in ft per second squared

t = 0:0.01:0.2;
figure(1)
for i = 1:6
    x_a = x_a_list(i)
    %% Find the state space model for the lofi model at the desired alt and vel.
    %%
    trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
    operating_point = operpoint('LIN_F16Block'); % retrieves initial conditions from integrators
    operating_point.Inputs(1).u = trim_thrust_lin; operating_point.Inputs(2).u = trim_control_lin(1);
    operating_point.Inputs(3).u = trim_control_lin(2); operating_point.Inputs(4).u = trim_control_lin(3);
    
    SS_lo = linearize('accelerometer_LIN_F16Block');
    
    all_tfs = tf(SS_lo);
    de2na_tf = minreal(all_tfs(end,2)) % this is the transfer function for elevator-to-normal-acc 
    
    %% Making the step response plots of elevator to n_a 
    %%
    opt = stepDataOptions('StepAmplitude',-1);
    [y,t] = step(de2na_tf,opt,t);
    
    string = join(['x_a = ', num2str(x_a), 'ft'])
    plot(t,y, 'DisplayName', string, 'LineWidth', 2);
    hold on 
    %% finding the zeros of the de2na tf
    %%

    Z = zero(de2na_tf)
end
hold off
grid
legend
xlabel('Time [s]')
ylabel('n_a[ft/s^2]')