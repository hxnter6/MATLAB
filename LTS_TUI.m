%%
% LTS_TUI.m
% SCoats
% 24/7/2018
% Text user interface for quasi-steady state point mass lap time simulation

% !!READ USER GUIDE AND AGREEMENT BEFORE USE!!
%%  =========================== Setup =====================================
warning off
clc; 
clear all; 
close all;

fprintf('LTS Coats 1.4\n\nPatience is a virtue.\n\n')

% Establish Global Varibles
Establish_Global

% Load Trackmap
addpath('Tracks')
% Ice_Cream_Cone % Preferred
% Ice_Cream_Cone_2 % Preferred (Matches the average speed of Lincoln 2018 for 1 lap)
% FSG_Endurance % Pretty Shitty
% Lincoln_2018
%Lincoln_2018_V2

%%  ====================== Simulation Parameters ==========================

% Simulation Dimension and Parameter Steps
batch = 0; % Dimensions of simulation array
 %ind_var1 = [linspace(44,46,20)]; % 1st varible to iterate
 %ind_var2 = [linspace(124,135,20)]; % 2nd varible to iterate
 
% Varible names for plot output (should include units)
 %var1 = 'Drag (lbs)';
 %var2 = 'Downforce (lbs)';

% Simulation Tolerance
dt = 0.001; % Timestep
roundoff = 3;   % Digits after decimal for time roundoff
v_tol  = 0.001; % m/s Convergence tolerance for velocity
a_tol = 0.0001; % g Convergence tolerance for acceleration
%% ============================= Tire Model ===============================
% 12 coeff Pacejka '93 Model Coefficients

% Hoosier R25B 18x7.5x10 IA = 0deg P = 12psi
beta_10_R25B_lat = [1.3925,-0.0011,2.5986,61.8209,0.1960,3.6969e+05,-0.0793e-05,0.0007,0.0020,-0.0020,0.1653,-7.7303];
beta_10_R25B_lon = [1.2309,-0.0027,2.9719,1.1974,0.0596,4.6389e+04,-0.0140,11.4756,253.5530,138.2941,0.0003,-4.5750];

% Hoosier R25B 16x7.5x10 IA = 0deg P = 12psi
beta_10_R25B_16_lat = [1.497564443, -0.002272231, 2.840472825, 70.86957406, 0.222001058, 369690, -3.24E-07, 0.000447167, -0.003834695, 0.002574449, -0.079271384, -6.855006239];
beta_10_R25B_16_lon = [1.2309,-0.0027,2.9719,1.1974,0.0596,4.6389e+04,-0.0140,11.4756,253.5530,138.2941,0.0003,-4.5750];

% Tire Coefficient Assignment
tire_coeff_lat = beta_10_R25B_16_lat;
tire_coeff_lon = beta_10_R25B_16_lon;

% Grip Modification Factor
mulat = 0.65;
mulon = 0.65;

% Slip limits for tire model
SA_max = 15; %deg
SR_max = 0.25;
SA_res = SA_max/50; %deg
SR_res = SR_max/50;
SA_range = 0:SA_res:SA_max;
% r_tire = 8.8 * 0.0254; % Tire loaded radius (m)
r_tire = 15.657/2 * 0.0254; % Tire loaded radius (m)

% Velocity Range for vehicle sim (Starts at 5 m/s because of extreme torque curve nonlinearity at low RPM)
v_min = 5.25;   % Lower velocity range for vehicle sim
v_max = 35.0;   % Upper velocity range for vehicle sim
dv = 0.05;      % Velocity differential for vehicle sim
v_range = v_min:dv:v_max;
enduro_laps = round(22000/sum(d));

%% ========================== Engine Model ================================
addpath('Engines')
%Engine = 'WEMS Yamaha R6 4-Speed E85 2019 Engine Dyno' ;
Motor = 'Emrax 208 HV' ;
Enginecode =[Motor '.csv'];
fprintf('Engine Selected: %s\n', Motor)

Solver_Output_Setup

for iii = 1:i_max
    for jjj = 1:j_max
        run = run + 1;
        fprintf('-------------------------------------\n')
        fprintf('Simulating Setup %2.0f/%2.0f\n',run,i_max*j_max)
        
        %% =========================== Car Setup ==========================
        
        %% Chassis Setuphich
        
        m_driver = 180 * 0.453592;  % Driver weight (lbs)
        m_car = 280 * 0.453592;     %lbs
        m_accum = 60 * 0.453592;    %lbs

        m_DRS = 1 * 0.453592;     %lbs

        track = 47 * 0.0254;    % Track
        l = 60.25 * 0.0254;     % Wheelbase
        cg_h = 11.2 * 0.0254; % CG height
        wdf = 0.445; % Weight Fraction Front
        finaldrive = 4;  
        
        %% Aerodynamics Setup
        adf = 0.46;                       % Downforce Fraction on Front in percent (0-1)
        Fl =  111 * 4.44822;              % lbf to N of down force @ 55 kph
        Fl_base = 111 * 4.44822;
        Fd = 45 * 4.44822;                % lbf to N of drag force @ 55 kph
        Fd_base = 45 * 4.44822;
        area = 1.15;                      % m^2 car frontal area
        rho = 1.204;
        drs_on = 1;
        drs_downforce_loss = 0.1;         % REeduction due to DRS Being active
        drs_drag_reduction = 0.3;         % Reduction due to DRS beign active

        % kg/m^3 air density
        %% Enable Drs by changing yes to 0, 1 for no drs.
        yes = 0
        if drs_on
            Fd = Fd_base * (1-drs_drag_reduction);  %Reduces Drag force according to DRS drag reductionb
            Fl = Fl_base * (1-drs_downforce_loss);   % Reduces Down force Accroding to DRS Downforce Reduction
       else
            Fd = Fd_base;                 % drs drag reduction percent (0-1)
            Fl = Fl_base;
    end

        %% Engine Setup
        % power_coeff = 0.8; % 2018 R6
        % power_coeff = 0.9; % FZ07
%         power_coeff = 0.7; % Empirical
          power_coeff = 1;
        shift_time = 0.1; %s          
        rpm_limit = 5500;
        %% Analysis Setup
        
        % Analysis Variables Axis 1
%       m_driver                = ind_var1(iii)* 0.453592; % lbs 
%       m_car                   = ind_var1(iii)* 0.453592; % lbs
%       m_accum                   = 65* 0.453592; % lbs
%       m_DRS                   = ind_var1(iii)* 0.453592; % lbs 
%       m_total                 = ind_var1(iii)* 0.453592; % lbs 
%       track                   = ind_var1(iii) * 0.0254;  % Track
%       l                       = ind_var1(iii) * 0.0254;  % Wheelbase
%       cg_h                    = ind_var1(iii) * 0.0254;  % M
%        wdf                     = ind_var1(iii);           % Weight Fraction Front
%       finaldrive              = ind_var1(iii);           % Ratio
%       tire_coefficient_lat    = ind_var1(iii,:);         % Fuck
%       adf                     = ind_var1(iii);           % Downforce Fraction on Front in percent (0-1)
%       Fl                      = ind_var1(iii) * 4.44822; % Downforce @ 55 kph
%       Fd                      = ind_var1(iii) * 4.44822; % Drag @ 55 kph
%       c_lift                  = ind_var1(iii);           % Coefficient of Lift
%       c_drag                  = ind_var1(iii);           % Coefficient of Drag
%       power_coeff             = ind_var1(iii);           % Coefficient of Power?
        
        % Analysis Variables Axis 2
%       m_driver                = ind_var2(jjj)* 0.453592; % lbs 
%       m_car                   = ind_var2(jjj)* 0.453592; % lbs
%       m_DRS                   = ind_var2(jjj)* 0.453592; % lbs 
%       m_total                 = ind_var2(jjj)* 0.453592; % lbs 
%       track                   = ind_var2(jjj) * 0.0254;  % Track
%       l                       = ind_var2(jjj) * 0.0254;  % Wheelbase
%       cg_h                    = ind_var2(jjj) * 0.0254;  % M
%       wdf                     = ind_var2(jjj);           % Weight Fraction Front
%       finaldrive              = ind_var2(jjj);           % Ratio
%       tire_coefficient_lat    = ind_var2(jjj,:);         % Fuck
%        adf                     = ind_var2(jjj);           % Downforce Fraction on Front in percent (0-1)
%       Fl                      = ind_var2(jjj) * 4.44822; % Downforce @ 55 kph
%       Fd                      = ind_var2(jjj) * 4.44822; % Drag @ 55 kph
%       c_lift                  = ind_var2(jjj);           % Coefficient of Lift
%       c_drag                  = ind_var2(jjj);           % Coefficient of Drag
%       power_coeff             = ind_var2(jjj);           % Coefficient of Power?

        % Converting "lift"/drag forces to coefficients for vehicle sim
        %c_lift = 2*Fl/(rho*area*15.6464^2);
        c_lift = (2*Fl/(rho*area*15.6464^2));
        %c_lift = (2*Fl/(rho*area*15.6464^2)*(1-drs_downforce_loss * drs));
        %c_drag = 2*Fd/(rho*area*15.6464^2);
        %Enable below for drs:
        c_drag = (2*Fd/(rho*area*15.6464^2));
        lift = @(v) 0.5*c_lift*rho*area*v^2;
        drag = @(v) 0.5*c_drag*rho*area*v^2;
        
        m_drag = ((c_drag/1.6)-1)*25;
         m_total = (m_car+m_driver+m_DRS+m_accum);
        %m_total = (m_car+m_driver+m_accum);
         %cg_h = (m_DRS/m_total) * (cg_h_DRS - cg_h) + cg_h;
        a = l*(1-wdf);
        b = l*wdf;
                 
        Engine_Force_Curves
%         peak_torque(jjj) = max(torque);
%         peak_power(jjj) = max(power);
        %% ============================Simulation==================================

        Clear_vars
        Vehicle_Sim_nlin
        Accel_sim
        Skidpad_sim
        Maneuver_Sim
        Post_Processing
    end
end

% Point calculations are done using the 2018 FSG rules.
Point_Calculation

%% ==============================Output====================================

% To be commented out for all non-engine analysis. Used to convert
% power/torque scaling coefficient to useful numbers
%
% ind_var2 = peak_power;

Plot_Results

%% =============================The End======================================