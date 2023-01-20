function [trim_state, trim_thrust, trim_control, dLEF, xu] = trim_F16(thrust, elevator, alpha, ail, rud, vel, alt, FC_flag)
%================================================
%     F16 nonlinear model trimming routine
%  for longitudinal motion, steady level flight
%
% Author: T. Keviczky
% Date:   April 29, 2002
%
%
%      Added addtional functionality.
%      This trim function can now trim at three 
%      additional flight conditions
%         -  Steady Turning Flight given turn rate
%         -  Steady Pull-up flight - given pull-up rate
%         -  Steady Roll - given roll rate
%
% Coauthor: Richard S. Russell
% Date:     November 7th, 2002
%
%
%================================================

global altitude velocity fi_flag_Simulink
global phi psi p q r phi_weight theta_weight psi_weight

altitude = alt;
velocity = vel;
alpha = alpha*pi/180;  %convert to radians

% OUTPUTS: trimmed values for states and controls
% INPUTS:  guess values for thrust, elevator, alpha  (assuming steady level flight)

% Initial Guess for free parameters
UX0 = [thrust; elevator; alpha; ail; rud];  % free parameters: two control values & angle of attack

% Initialize some varibles
%
phi = 0; psi = 0;
p = 0; q = 0; r = 0;
phi_weight = 10; theta_weight = 10; psi_weight = 10;

if ~exist('FC_flag','var')
  % No flight condition provided, ask the user
    disp('At what flight condition would you like to trim the F-16?');
    disp('1.  Steady Wings-Level Flight.');
    disp('2.  Steady Turning Flight.');
    disp('3.  Steady Pull-Up Flight.');
    disp('4.  Steady Roll Flight.');
    FC_flag = input('Your Selection:  ');
end

switch FC_flag
    case 1
        % do nothing
    case 2
        % This case doesn't make a lot of sense at the moment, as phi and 
        % q are fixed at zero
        r = input('Enter the turning rate (deg/s):  ');
        psi_weight = 0;
    case 3
        q = input('Enter the pull-up rate (deg/s):  ');
        theta_weight = 0;
    case 4    
        p = input('Enter the Roll rate    (deg/s):  ');
        phi_weight = 0;
    otherwise
        disp('Invalid Selection')
%        break;
end

% Initializing optimization options and running optimization:
OPTIONS = optimset('TolFun',1e-10,'TolX',1e-10,'MaxFunEvals',5e+04,'MaxIter',1e+04);

iter = 1;
yes = 0;
while iter == 1
   
    % Constraints on [thrust, elevator, alpha, aileron, rudder]
    if fi_flag_Simulink == 0
        ub = [19000; 25; 45*pi/180; 21.5; 30];
        lb = [1000; -25; -10*pi/180; -21.5; -30];
    else
        ub = [19000; 25; 90*pi/180; 21.5; 30];
        lb = [1000; -25; -20*pi/180; -21.5; -30];
    end
    UX0 = lb .* (UX0 < lb) + UX0 .* ~(UX0 < lb);
    UX0 = ub .* (UX0 > ub) + UX0 .* ~(UX0 > ub);
    
    % fminsearch can get stuck in a local minimum. Using a different initial
    % value may help in that case. 
    [UX,FVAL,EXITFLAG,OUTPUT] = fminsearch('trimfun',UX0,OPTIONS);
   
    [cost, Xdot, xu] = trimfun(UX);
    
%     disp('Trim Values and Cost:');
    disp('Trim Cost:');
    disp(['cost   = ' num2str(cost)])
    disp(['thrust = ' num2str(xu(13)) ' lb'])
    disp(['elev   = ' num2str(xu(14)) ' deg'])
    disp(['ail    = ' num2str(xu(15)) ' deg'])
    disp(['rud    = ' num2str(xu(16)) ' deg'])
    disp(['alpha  = ' num2str(xu(8)*180/pi) ' deg'])
    disp(['dLEF   = ' num2str(xu(17)) ' deg'])
    disp(['Vel.   = ' num2str(velocity) 'ft/s']) 
%     flag = input('Continue trim routine iterations? (y/n):  ','s'); 
    disp('Continue trim routine iterations? (y/n):  '); 
    if yes < 3
        flag = 'y';
        disp(['y' newline 'Continue trimming ' num2str(yes) newline]);
    elseif yes >= 3
        flag = 'n';
        disp(['n' newline 'End trimming ' num2str(yes) newline '----------------------------------------']);
    end

    if flag == 'n'
        iter = 0;
    end
    UX0 = UX;
    yes = yes + 1;
end

% For simulink:
trim_state=xu(1:12);
trim_thrust=UX(1);
trim_ele=UX(2);
trim_ail=UX(4);
trim_rud=UX(5);
trim_control=[UX(2);UX(4);UX(5)];
dLEF = xu(17);