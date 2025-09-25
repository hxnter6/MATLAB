%% LTS_Complete_Standalone.m
% Comprehensive Lap Time Simulation (LTS) for Formula SAE Vehicles
% Author: Consolidated from SCoats LTS Coats 1.4
% Date: Created from multiple source files
%
% This standalone MATLAB file contains all functionalities for:
% - Quasi-steady state point mass vehicle simulation
% - Pacejka '93 tire model implementation  
% - Aerodynamics with DRS (Drag Reduction System)
% - Engine/Motor force curves with gear shifting
% - Track simulation (corners and straights)
% - Performance event simulations (Autocross, Endurance, Skidpad, Acceleration)
% - FSG scoring system implementation
% - Comprehensive plotting and visualization
%
% Usage: Simply run this file in MATLAB. All parameters can be adjusted
%        in the "Vehicle Setup" and "Simulation Parameters" sections below.

%% ========================== INITIALIZATION ==============================
warning off;
clc; 
clear all; 
close all;

fprintf('LTS Complete Standalone - Comprehensive Formula SAE Lap Time Simulation\n');
fprintf('Initializing simulation...\n\n');

%% ========================== SIMULATION PARAMETERS =======================

% Time stepping and convergence tolerances
dt = 0.001;                 % Timestep (s)
roundoff = 3;               % Digits after decimal for time roundoff
v_tol = 0.001;              % m/s Convergence tolerance for velocity  
a_tol = 0.0001;             % g Convergence tolerance for acceleration

% Velocity range for vehicle simulation
v_min = 5.25;               % Lower velocity range (m/s)
v_max = 35.0;               % Upper velocity range (m/s) 
dv = 0.05;                  % Velocity differential (m/s)
v_range = v_min:dv:v_max;

% Constants
g = 9.81;                   % Gravitational acceleration (m/s²)

%% ========================== TIRE MODEL PARAMETERS =======================

% 12 coefficient Pacejka '93 Model Coefficients
% Hoosier R25B 16x7.5x10 IA = 0deg P = 12psi
tire_coeff_lat = [1.497564443, -0.002272231, 2.840472825, 70.86957406, 0.222001058, 369690, -3.24E-07, 0.000447167, -0.003834695, 0.002574449, -0.079271384, -6.855006239];
tire_coeff_lon = [1.2309,-0.0027,2.9719,1.1974,0.0596,4.6389e+04,-0.0140,11.4756,253.5530,138.2941,0.0003,-4.5750];

% Grip modification factors
mulat = 0.65;               % Lateral grip multiplier
mulon = 0.65;               % Longitudinal grip multiplier

% Slip limits and resolution for tire model
SA_max = 15;                % Maximum slip angle (deg)
SR_max = 0.25;              % Maximum slip ratio
SA_res = SA_max/50;         % Slip angle resolution (deg)
SR_res = SR_max/50;         % Slip ratio resolution
SA_range = 0:SA_res:SA_max; % Slip angle range

% Tire physical properties
r_tire = 15.657/2 * 0.0254; % Tire loaded radius (m)

%% ========================== VEHICLE SETUP ===============================

% Mass properties (converted from lbs to kg)
m_driver = 180 * 0.453592;   % Driver mass (kg)
m_car = 280 * 0.453592;      % Car mass (kg) 
m_accum = 60 * 0.453592;     % Accumulator mass (kg)
m_DRS = 1 * 0.453592;        % DRS system mass (kg)
m_total = m_car + m_driver + m_accum + m_DRS; % Total mass (kg)

% Geometry (converted from inches to meters)
track = 47 * 0.0254;         % Track width (m)
l = 60.25 * 0.0254;          % Wheelbase (m)
cg_h = 11.2 * 0.0254;        % CG height (m)
wdf = 0.445;                 % Weight distribution front (fraction)
a = l * (1-wdf);             % Distance from CG to front axle (m)
b = l * wdf;                 % Distance from CG to rear axle (m)

% Aerodynamics setup
area = 1.15;                 % Frontal area (m²)
rho = 1.204;                 % Air density (kg/m³)
adf = 0.46;                  % Downforce distribution front (fraction)

% Base aerodynamic forces at reference speed (55 kph = 15.6464 m/s)
Fl_base = 111 * 4.44822;     % Base downforce (N)
Fd_base = 45 * 4.44822;      % Base drag (N)

% DRS (Drag Reduction System) parameters
drs_drag_reduction = 0.30;   % Drag reduction when DRS active (fraction)
drs_downforce_loss = 0.10;   % Downforce loss when DRS active (fraction)

% Engine/Motor setup
power_coeff = 1.0;           % Power scaling coefficient
shift_time = 0.1;            % Shift time (s)
rpm_limit = 5500;            % RPM limit
finaldrive = 4;              % Final drive ratio

%% ========================== SAMPLE TRACK DEFINITION =====================
% Simple test track - Ice Cream Cone inspired layout
% r = radius of curvature (0 = straight), d = distance

fprintf('Loading sample track (Ice Cream Cone layout)...\n');

% Track segments: [radius, distance] 
% Radius = 0 for straights, positive for corners
track_segments = [
    0,    30;    % Start straight
    15,   23.56; % Corner 1 (90 degrees, r=15m)
    0,    40;    % Straight
    12,   18.85; % Corner 2 (90 degrees, r=12m) 
    0,    35;    % Straight
    20,   31.42; % Corner 3 (90 degrees, r=20m)
    0,    50;    % Straight
    18,   28.27; % Corner 4 (90 degrees, r=18m)
    0,    45;    % Back straight
];

r = track_segments(:,1)';    % Radius array
d = track_segments(:,2)';    % Distance array
distance_total = sum(d);     % Total track distance

fprintf('Track loaded: %d segments, total distance %.1f m\n', numel(d), distance_total);

% Calculate endurance laps (22km total distance)
enduro_laps = round(22000/distance_total);

%% ========================== SAMPLE ENGINE DATA ==========================
% Sample motor data - Emrax 208 HV characteristics
fprintf('Loading sample motor data (Emrax 208 HV)...\n');

% RPM range
engine_spd = 0:100:5500;    % RPM range

% Power curve (HP) - simplified characteristic curve
power_hp = zeros(size(engine_spd));
for i = 1:length(engine_spd)
    rpm = engine_spd(i);
    if rpm <= 1000
        power_hp(i) = rpm * 0.08;           % Linear rise
    elseif rpm <= 3000
        power_hp(i) = 80 + (rpm-1000) * 0.02;  % Gradual increase
    elseif rpm <= 4500
        power_hp(i) = 120 - (rpm-3000) * 0.01; % Peak region
    else
        power_hp(i) = 105 - (rpm-4500) * 0.02; % Drop off
    end
end

% Torque curve (ft-lbs) - calculated from power
torque_ftlbs = zeros(size(engine_spd));
for i = 1:length(engine_spd)
    if engine_spd(i) > 0
        torque_ftlbs(i) = power_hp(i) * 5252 / engine_spd(i);
    else
        torque_ftlbs(i) = 200; % Starting torque
    end
end

% Convert to SI units
power = power_hp * 0.745699872 * power_coeff;  % Convert to kW
torque = torque_ftlbs * 1.35582 * power_coeff; % Convert to N⋅m

% Simple single-speed gearing for electric motor
gearing = [1, 3.5]; % [primary, final gear ratio]
primary = gearing(1);
gear_ratio = gearing(2);

fprintf('Motor data loaded: Max power %.1f kW, Max torque %.1f Nm\n', max(power), max(torque));

%% ========================== FUNCTION DEFINITIONS ========================

%% ========================== HELPER FUNCTIONS ============================
% All helper functions are defined at the end of this file for proper scoping

%% ========================== MOTOR FORCE CURVES ==========================
fprintf('Generating motor force curves...\n');

% Calculate engine force at each velocity
Fx_engine = zeros(size(v_range));
gear_pos = ones(size(v_range));  % Single gear for electric motor

for ii = 1:length(v_range)
    v = v_range(ii);
    
    % Calculate RPM from vehicle speed
    rpm_calc = round2((v * primary * gear_ratio * finaldrive * (60/(2*pi)) / r_tire), 100);
    
    % Find closest RPM in lookup table
    [~, ind] = min(abs(engine_spd - rpm_calc));
    
    % Check RPM limit
    if rpm_calc > rpm_limit
        Fx_engine(ii) = 0;
    else
        % Calculate force at tire contact patch
        Fx_engine(ii) = torque(ind) * primary * gear_ratio * finaldrive / r_tire;
    end
end

fprintf('Motor force curves generated.\n');

%% ========================== VEHICLE MODEL GENERATION ====================
fprintf('Generating vehicle performance model...\n');

% Initialize output arrays
Ay_out = zeros(1, numel(v_range));
Ax_drive_out = zeros(1, numel(v_range));
Ax_brake_out = zeros(1, numel(v_range));
alpha_out = zeros(numel(v_range), 4);

% Generate aerodynamic functions (without DRS for now)
c_lift = 2*Fl_base/(rho*area*15.6464^2);
c_drag = 2*Fd_base/(rho*area*15.6464^2);
lift = @(v) 0.5*c_lift*rho*area*v.^2;
drag = @(v) 0.5*c_drag*rho*area*v.^2;

% Calculate vehicle performance at each velocity
for i = 1:numel(v_range)
    v = v_range(i);
    
    % Aerodynamic forces
    fdowns = lift(v);  % Downforce (N)
    fdrag = drag(v);   % Drag (N)
    
    % Static corner weights (N)
    w_1 = 0.5 * m_total * g * wdf;      % Left front
    w_2 = 0.5 * m_total * g * wdf;      % Right front  
    w_3 = 0.5 * m_total * g * (1-wdf);  % Left rear
    w_4 = 0.5 * m_total * g * (1-wdf);  % Right rear
    
    %% LATERAL ACCELERATION LIMIT
    Ay_in = 0; Ay_last = 10;
    
    while abs(Ay_in - Ay_last) >= a_tol
        Ay_last = Ay_in;
        
        % Load transfer due to lateral acceleration
        LT_f = Ay_last * m_total * g * cg_h * wdf / track;
        LT_r = Ay_last * m_total * g * cg_h * (1-wdf) / track;
        
        % Calculate path radii for each wheel
        R1 = (v^2)/(Ay_last*g) - track/2;  % Left front
        R2 = (v^2)/(Ay_last*g) + track/2;  % Right front
        R3 = (v^2)/(Ay_last*g) - track/2;  % Left rear
        R4 = (v^2)/(Ay_last*g) + track/2;  % Right rear
        
        % Slip angles due to vehicle geometry
        heading1 = atand(l*(1-wdf)/R1);
        heading2 = atand(l*(1-wdf)/R2);
        heading3 = atand(l*wdf/R3);
        heading4 = atand(l*wdf/R4);
        
        SA_delta_f = heading1 - heading2;
        SA_delta_r = heading3 - heading4;
        
        % Normal loads on each wheel (convert to lbf for tire model)
        Fz_1 = (w_1 + 0.5*fdowns*adf - LT_f) / 4.44822;
        Fz_2 = (w_2 + 0.5*fdowns*adf + LT_f) / 4.44822;
        Fz_3 = (w_3 + 0.5*fdowns*(1-adf) - LT_r) / 4.44822;
        Fz_4 = (w_4 + 0.5*fdowns*(1-adf) + LT_r) / 4.44822;
        
        % Handle wheel lift-off
        if Fz_1 < 0
            Fz_2 = Fz_2 - Fz_1;
            Fz_1 = 0;
        end
        if Fz_3 < 0
            Fz_4 = Fz_4 - Fz_3;
            Fz_3 = 0;
        end
        
        % Calculate maximum lateral forces
        [Fy_1, alpha_1] = Fy_max_func(Fz_1, SA_max, SA_res);
        [Fy_2, alpha_2] = Fy_max_func(Fz_2, SA_max, SA_res);
        
        % Rear axle with slip angle difference
        Fy_range_3 = Fy_range_func(Fz_3, SA_range + SA_delta_r);
        Fy_range_4 = Fy_range_func(Fz_4, SA_range);
        [Fy_rear, SA_r] = max(Fy_range_3 + Fy_range_4);
        Fy_3 = Fy_range_3(SA_r);
        Fy_4 = Fy_range_4(SA_r);
        alpha_3 = SA_range(SA_r) + SA_delta_r;
        alpha_4 = SA_range(SA_r);
        
        % Moment balance check
        Mz_cg_F = (Fy_1 + Fy_2) * (1-wdf);
        Mz_cg_R = (Fy_3 + Fy_4) * wdf;
        
        if Mz_cg_F > Mz_cg_R
            Fy_F_scale = Mz_cg_R / Mz_cg_F;
            Fy_R_scale = 1;
        elseif Mz_cg_F < Mz_cg_R
            Fy_R_scale = Mz_cg_F / Mz_cg_R;
            Fy_F_scale = 1;
        else
            Fy_F_scale = 1;
            Fy_R_scale = 1;
        end
        
        % Total lateral acceleration capability
        total_Fy = (Fy_1*Fy_F_scale + Fy_2*Fy_F_scale + Fy_3*Fy_R_scale + Fy_4*Fy_R_scale) * 4.44822;
        Ay_in = total_Fy / (m_total * g);
    end
    
    alpha_out(i,:) = [alpha_1, alpha_2, alpha_3, alpha_4];
    Ay_out(i) = Ay_in * g;
    
    %% DRIVING ACCELERATION LIMIT
    Ax_drive_in = 0; Ax_last = 10;
    
    while abs(Ax_drive_in - Ax_last) > a_tol
        Ax_last = Ax_drive_in;
        
        % Load transfer due to longitudinal acceleration
        LT = g * m_total * Ax_last * cg_h / (l * 2);
        
        % Normal loads (driving - rear wheels only)
        Fz_3_drive = (w_3 + 0.5*fdowns*(1-adf) + LT) / 4.44822;
        Fz_4_drive = (w_4 + 0.5*fdowns*(1-adf) + LT) / 4.44822;
        
        % Maximum driving forces
        SR = 0:SR_res:SR_max;
        Fx_3_array = zeros(size(SR));
        for sr_i = 1:length(SR)
            Fx_3_array(sr_i) = pacejka_tire_model(tire_coeff_lon, [Fz_3_drive, SR(sr_i), mulon]);
        end
        Fx_3 = max(Fx_3_array);
        
        Fx_4_array = zeros(size(SR));
        for sr_i = 1:length(SR)
            Fx_4_array(sr_i) = pacejka_tire_model(tire_coeff_lon, [Fz_4_drive, SR(sr_i), mulon]);
        end
        Fx_4 = max(Fx_4_array);
        
        Fx_tire = (Fx_3 + Fx_4) * 4.44822;
        
        % Limit by tire or engine capability
        if Fx_tire >= Fx_engine(i)
            Ax_drive_in = (Fx_engine(i) - fdrag) / (m_total * g);
        else
            Ax_drive_in = (Fx_tire - fdrag) / (m_total * g);
        end
    end
    
    Ax_drive_out(i) = Ax_drive_in * g;
    
    %% BRAKING ACCELERATION LIMIT  
    Ax_brake_in = 0; Ax_last = 10;
    
    while abs(Ax_brake_in - Ax_last) > a_tol
        Ax_last = Ax_brake_in;
        
        % Load transfer due to braking
        LT = g * m_total * Ax_last * cg_h / (l * 2);
        
        % Normal loads (all wheels brake)
        Fz_1_brake = (w_1 + 0.5*fdowns*adf - LT) / 4.44822;
        Fz_2_brake = (w_2 + 0.5*fdowns*adf - LT) / 4.44822;
        Fz_3_brake = (w_3 + 0.5*fdowns*(1-adf) + LT) / 4.44822;
        Fz_4_brake = (w_4 + 0.5*fdowns*(1-adf) + LT) / 4.44822;
        
        % Maximum braking forces
        Fx_1 = -Fx_brake_func(Fz_1_brake, SR_max, SR_res);
        Fx_2 = -Fx_brake_func(Fz_2_brake, SR_max, SR_res);
        Fx_3 = -Fx_brake_func(Fz_3_brake, SR_max, SR_res);
        Fx_4 = -Fx_brake_func(Fz_4_brake, SR_max, SR_res);
        
        Fx_tire = (Fx_1 + Fx_2 + Fx_3 + Fx_4) * 4.44822;
        
        Ax_brake_in = (Fx_tire - fdrag) / (m_total * g);
    end
    
    Ax_brake_out(i) = Ax_brake_in * g;
end

% Create polynomial fits for smooth interpolation
P1 = polyfit(v_range, Ay_out, 3);
P2 = polyfit(v_range, Ax_drive_out, 10);
P3 = polyfit(v_range, Ax_brake_out, 3);

% Create function handles
Ay = @(x) P1(1).*x.^3 + P1(2).*x.^2 + P1(3).*x + P1(4);
Ax_drive = @(x) P2(1).*x.^10 + P2(2).*x.^9 + P2(3).*x.^8 + P2(4).*x.^7 + P2(5).*x.^6 + P2(6).*x.^5 + P2(7).*x.^4 + P2(8).*x.^3 + P2(9).*x.^2 + P2(10).*x.^1 + P2(11);
Ax_brake = @(x) P3(1).*x.^3 + P3(2).*x.^2 + P3(3).*x + P3(4);

fprintf('Vehicle model generated successfully.\n');

%% ========================== LAP SIMULATION ===============================
fprintf('Running lap simulation...\n');

% Initialize output matrices
time_out = zeros(numel(d), 20000);
v_out = zeros(numel(d), 20000);
throttle_out = zeros(numel(d), 20000);

% Simulate each track segment
for i = 1:numel(d)
    if r(i) > 0  % Corner segment
        % Find steady-state cornering velocity
        v_check = 0; corner_conv = 0;
        
        while corner_conv ~= 1
            [v_c, time_c, throttle_c] = corner_sim(Ay(v_check), r(i), d(i), roundoff, dt, 0);
            v_i = mean(v_c);
            
            if abs(v_check - v_i) >= v_tol
                v_check = abs(v_check + v_i) / 2;
            else
                corner_conv = 1;
            end
        end
        
        % Store corner results
        for j = 1:numel(time_c)
            time_out(i,j) = time_c(j);
            v_out(i,j) = min(v_c(j), v_max);
            throttle_out(i,j) = throttle_c(j);
        end
        
    else  % Straight segment
        % Determine entry and exit velocities from adjacent corners
        if i == 1  % First segment
            % Get exit velocity from last corner
            v_check = 0; corner_conv = 0;
            while corner_conv ~= 1
                [v_last, ~, ~] = corner_sim(Ay(v_check), r(end), d(end), roundoff, dt, 0);
                v_i = mean(v_last);
                if abs(v_check - v_i) >= v_tol
                    v_check = abs(v_check + v_i) / 2;
                else
                    corner_conv = 1;
                end
            end
            v_exit = mean(v_last);
            
            % Get entry velocity from next corner
            v_check = 0; corner_conv = 0;
            while corner_conv ~= 1
                [v_next, ~, ~] = corner_sim(Ay(v_check), r(i+1), d(i+1), roundoff, dt, 0);
                v_i = mean(v_next);
                if abs(v_check - v_i) >= v_tol
                    v_check = abs(v_check + v_i) / 2;
                else
                    corner_conv = 1;
                end
            end
            v_entry = mean(v_next);
            
        elseif i == numel(d)  % Last segment
            % Get exit velocity from previous corner
            v_check = 0; corner_conv = 0;
            while corner_conv ~= 1
                [v_last, ~, ~] = corner_sim(Ay(v_check), r(i-1), d(i-1), roundoff, dt, 0);
                v_i = mean(v_last);
                if abs(v_check - v_i) >= v_tol
                    v_check = abs(v_check + v_i) / 2;
                else
                    corner_conv = 1;
                end
            end
            v_exit = mean(v_last);
            
            % Get entry velocity from first corner (lap closure)
            v_check = 0; corner_conv = 0;
            while corner_conv ~= 1
                [v_next, ~, ~] = corner_sim(Ay(v_check), r(1), d(1), roundoff, dt, 0);
                v_i = mean(v_next);
                if abs(v_check - v_i) >= v_tol
                    v_check = abs(v_check + v_i) / 2;
                else
                    corner_conv = 1;
                end
            end
            v_entry = mean(v_next);
            
        else  % Middle segments
            % Get exit velocity from previous corner
            v_check = 0; corner_conv = 0;
            while corner_conv ~= 1
                [v_last, ~, ~] = corner_sim(Ay(v_check), r(i-1), d(i-1), roundoff, dt, 0);
                v_i = mean(v_last);
                if abs(v_check - v_i) >= v_tol
                    v_check = abs(v_check + v_i) / 2;
                else
                    corner_conv = 1;
                end
            end
            v_exit = mean(v_last);
            
            % Get entry velocity from next corner
            v_check = 0; corner_conv = 0;
            while corner_conv ~= 1
                [v_next, ~, ~] = corner_sim(Ay(v_check), r(i+1), d(i+1), roundoff, dt, 0);
                v_i = mean(v_next);
                if abs(v_check - v_i) >= v_tol
                    v_check = abs(v_check + v_i) / 2;
                else
                    corner_conv = 1;
                end
            end
            v_entry = mean(v_next);
        end
        
        % Simulate straight line performance
        [time_s, v_s, throttle_s] = straight_sim(d(i), dt, Ax_drive, Ax_brake, v_exit, v_entry, v_tol);
        
        % Store straight results
        for k = 1:numel(time_s)
            time_out(i,k) = time_s(k);
            v_out(i,k) = v_s(k);
            throttle_out(i,k) = throttle_s(k);
        end
    end
end

fprintf('Lap simulation completed.\n');

%% ========================== POST PROCESSING =============================
fprintf('Post processing results...\n');

% Convert time and velocity matrices to vectors
limits = size(time_out);
time_vec = time_out(1,:);
v_vec = v_out(1,:);
throttle_vec = throttle_out(1,:);
qqq = limits(2);
ref = 0;

% Flatten matrices into continuous vectors
for q = 1:limits(1)
    for qq = 1:limits(2)
        qqq = qqq + 1;
        time_vec(qqq) = time_out(q,qq);
        if time_vec(qqq) ~= 0
            time_vec(qqq) = time_out(q,qq) + ref;
        end
        v_vec(qqq) = v_out(q,qq);
        throttle_vec(qqq) = throttle_out(q,qq);
    end
    ref = max(time_out(q,:)) + ref;
end

% Extract non-zero values
zz = 0;
for z = 1:numel(time_vec)
    if time_vec(z) ~= 0
        zz = zz + 1;
        time(zz) = time_vec(z);
        speed(zz) = v_vec(z);
        if speed(zz) > v_max
            speed(zz) = v_max;
        end
        throttle(zz) = throttle_vec(z);
    end
end
speed(1) = speed(2);  % Fix first point

% Calculate lap time
laptime = max(time);
endurotime = laptime * enduro_laps;

fprintf('Post processing completed.\n');

%% ========================== ADDITIONAL SIMULATIONS ======================
fprintf('Running additional performance simulations...\n');

%% Acceleration Simulation (0-75m)
acceltime = 0;
w = 1;
d_accel(1) = 0;
xd_accel(1) = 5;  % Start at 5 m/s
xdd_accel(1) = 0;

while d_accel(w) <= 75  % 75m acceleration distance
    acceltime = acceltime + dt;
    xdd_accel(w+1) = abs(Ax_drive(xd_accel(w)));
    xd_accel(w+1) = xd_accel(w) + 0.5*dt*(xdd_accel(w+1) + xdd_accel(w));
    d_accel(w+1) = d_accel(w) + 0.5*dt*(xd_accel(w+1) + xd_accel(w));
    w = w + 1;
end

%% Skidpad Simulation
v_check = 0; corner_conv = 0;
r_skid = 7.625 + track/2;  % Skidpad radius

while corner_conv ~= 1
    [v_skid, time_skid, ~] = corner_sim(Ay(v_check), r_skid, r_skid*2*pi, roundoff, dt, 0);
    v_i = mean(v_skid);
    if abs(v_check - v_i) >= v_tol
        v_check = abs(v_check + v_i) / 2;
    else
        corner_conv = 1;
    end
end

skidpadtime = max(time_skid);

fprintf('Additional simulations completed.\n');

%% ========================== SCORING CALCULATION =========================
fprintf('Calculating competition scores...\n');

% Assume these are the best times for scoring reference
tbest = laptime;
skidbest = skidpadtime;
accelbest = acceltime;
endurobest = endurotime;

% Calculate scores using FSG rules
autox_pts = 95.5 * ((((tbest*1.25)/laptime) - 1) / 0.25) + 4.5;
enduro_pts = 300 * ((((endurobest*1.333)/endurotime) - 1) / 0.333) + 25;
skidpad_pts = 71.5 * ((((skidbest*1.25)/skidpadtime)^2 - 1) / 0.5625) + 3.5;
accel_pts = 71.5 * (((accelbest*1.5/acceltime) - 1) / 0.5) + 3.5;

total_pts = autox_pts + enduro_pts + skidpad_pts + accel_pts;

fprintf('Scoring calculation completed.\n');

%% ========================== RESULTS DISPLAY =============================
fprintf('\n========================== SIMULATION RESULTS ==========================\n');
fprintf('Autocross Lap Time:    %5.3f s\n', laptime);
fprintf('Endurance Time:        %5.3f s\n', endurotime);
fprintf('Skidpad Time:          %5.3f s\n', skidpadtime);
fprintf('Acceleration Time:     %5.3f s\n', acceltime);
fprintf('Average Velocity:      %5.3f m/s\n', sum(speed.*dt)/max(time));
fprintf('\n========================== COMPETITION SCORES ==========================\n');
fprintf('Autocross Points:      %5.1f\n', autox_pts);
fprintf('Endurance Points:      %5.1f\n', enduro_pts);
fprintf('Skidpad Points:        %5.1f\n', skidpad_pts);
fprintf('Acceleration Points:   %5.1f\n', accel_pts);
fprintf('Total Dynamic Points:  %5.1f\n', total_pts);
fprintf('=========================================================================\n');

%% ========================== PLOTTING RESULTS ============================
fprintf('Generating plots...\n');

% Time vs Speed plot with throttle coloring
figure('Name', 'Lap Time vs Speed', 'Position', [100, 100, 800, 600]);
scatter(time, speed, 4, throttle, 'filled');
grid on;
colormap jet;
colorbar;
title('Lap Time vs Vehicle Speed (colored by throttle position)');
xlabel('Time (s)');
ylabel('Speed (m/s)');

% Vehicle performance curves
figure('Name', 'Vehicle Performance Envelopes', 'Position', [200, 150, 1200, 800]);

subplot(2,2,1);
plot(v_range, Ay_out/g, 'b-', 'LineWidth', 2);
hold on;
plot(v_range, Ay(v_range)/g, 'r--', 'LineWidth', 1);
grid on;
title('Lateral Acceleration Capability');
xlabel('Velocity (m/s)');
ylabel('Lateral Acceleration (g)');
legend('Calculated Points', 'Polynomial Fit', 'Location', 'best');

subplot(2,2,2);
plot(v_range, Ax_drive_out/g, 'b-', 'LineWidth', 2);
hold on;
plot(v_range, Ax_drive(v_range)/g, 'r--', 'LineWidth', 1);
grid on;
title('Driving Acceleration Capability');
xlabel('Velocity (m/s)');
ylabel('Longitudinal Acceleration (g)');
legend('Calculated Points', 'Polynomial Fit', 'Location', 'best');

subplot(2,2,3);
plot(v_range, -Ax_brake_out/g, 'b-', 'LineWidth', 2);
hold on;
plot(v_range, -Ax_brake(v_range)/g, 'r--', 'LineWidth', 1);
grid on;
title('Braking Acceleration Capability');
xlabel('Velocity (m/s)');
ylabel('Braking Deceleration (g)');
legend('Calculated Points', 'Polynomial Fit', 'Location', 'best');

subplot(2,2,4);
plot(v_range, alpha_out(:,1), 'r-', 'LineWidth', 1.5);
hold on;
plot(v_range, alpha_out(:,2), 'g-', 'LineWidth', 1.5);
plot(v_range, alpha_out(:,3), 'b-', 'LineWidth', 1.5);
plot(v_range, alpha_out(:,4), 'm-', 'LineWidth', 1.5);
grid on;
title('Tire Slip Angles vs Speed');
xlabel('Velocity (m/s)');
ylabel('Slip Angle (degrees)');
legend('Left Front', 'Right Front', 'Left Rear', 'Right Rear', 'Location', 'best');

% Motor performance
figure('Name', 'Motor Performance', 'Position', [300, 200, 800, 600]);

subplot(2,1,1);
plot(engine_spd, power, 'r-', 'LineWidth', 2);
hold on;
plot(engine_spd, torque/10, 'b-', 'LineWidth', 2);  % Scale torque for visibility
grid on;
title('Motor Power and Torque Curves');
xlabel('Motor Speed (RPM)');
ylabel('Power (kW) / Torque/10 (Nm)');
legend('Power (kW)', 'Torque/10 (Nm)', 'Location', 'best');

subplot(2,1,2);
plot(v_range, Fx_engine/1000, 'g-', 'LineWidth', 2);
grid on;
title('Driving Force at Contact Patch');
xlabel('Vehicle Speed (m/s)');
ylabel('Driving Force (kN)');

% Track layout visualization (simplified)
figure('Name', 'Track Layout and Performance', 'Position', [400, 250, 1000, 700]);

% Create simple track visualization
x_track = 0; y_track = 0; heading = 0;
x_coords = []; y_coords = [];

for i = 1:numel(d)
    if r(i) == 0  % Straight
        x_end = x_track + d(i) * cos(heading);
        y_end = y_track + d(i) * sin(heading);
        x_coords = [x_coords, linspace(x_track, x_end, 20)];
        y_coords = [y_coords, linspace(y_track, y_end, 20)];
        x_track = x_end; y_track = y_end;
    else  % Corner
        arc_angle = d(i) / r(i);  % Total angle of corner
        angles = linspace(0, arc_angle, 20);
        center_x = x_track - r(i) * sin(heading);
        center_y = y_track + r(i) * cos(heading);
        
        for j = 1:length(angles)
            angle = heading + angles(j);
            x_coords = [x_coords, center_x + r(i) * sin(angle)];
            y_coords = [y_coords, center_y - r(i) * cos(angle)];
        end
        
        x_track = x_coords(end);
        y_track = y_coords(end);
        heading = heading + arc_angle;
    end
end

subplot(2,1,1);
plot(x_coords, y_coords, 'k-', 'LineWidth', 3);
axis equal; grid on;
title('Track Layout');
xlabel('Distance (m)');
ylabel('Distance (m)');

subplot(2,1,2);
distance_cumulative = cumsum([0, d]);
segment_speeds = zeros(size(d));
for i = 1:numel(d)
    non_zero_idx = find(v_out(i,:) > 0, 1, 'last');
    if ~isempty(non_zero_idx)
        segment_speeds(i) = mean(v_out(i,1:non_zero_idx));
    end
end

bar(1:numel(d), segment_speeds, 'FaceColor', [0.3, 0.7, 0.9]);
grid on;
title('Average Speed per Track Segment');
xlabel('Segment Number');
ylabel('Average Speed (m/s)');

% Performance summary
figure('Name', 'Performance Summary', 'Position', [500, 300, 800, 600]);

events = {'Autocross', 'Endurance', 'Skidpad', 'Acceleration'};
times = [laptime, endurotime, skidpadtime, acceltime];
points = [autox_pts, enduro_pts, skidpad_pts, accel_pts];

subplot(2,1,1);
bar(times, 'FaceColor', [0.8, 0.4, 0.2]);
set(gca, 'XTickLabel', events);
grid on;
title('Event Times');
ylabel('Time (s)');

subplot(2,1,2);
bar(points, 'FaceColor', [0.2, 0.8, 0.4]);
set(gca, 'XTickLabel', events);
grid on;
title('Event Points (FSG Scoring)');
ylabel('Points');

fprintf('All plots generated successfully.\n');

%% ========================== SIMULATION COMPLETE =========================
fprintf('\n========================== SIMULATION COMPLETE =========================\n');
fprintf('LTS Complete Standalone simulation finished successfully.\n');
fprintf('All vehicle performance data, lap simulation results, and plots are ready.\n');
fprintf('You can modify parameters in the setup sections and re-run for different configurations.\n');
fprintf('=========================================================================\n');

% Save results to workspace
save('LTS_Results.mat', 'laptime', 'endurotime', 'skidpadtime', 'acceltime', ...
     'autox_pts', 'enduro_pts', 'skidpad_pts', 'accel_pts', 'total_pts', ...
     'time', 'speed', 'throttle', 'v_range', 'Ay_out', 'Ax_drive_out', 'Ax_brake_out');

fprintf('Results saved to LTS_Results.mat\n');