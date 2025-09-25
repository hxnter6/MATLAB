%%
% LTS_Standalone.m - Complete Lap Time Simulation System
% SCoats - Standalone version with all functionality integrated
% 24/7/2018 - Updated and consolidated into single file
%
% This is a comprehensive standalone MATLAB file containing all the
% functionality of the LTS (Lap Time Simulation) system for Formula SAE/FSG
% vehicles. It includes vehicle dynamics modeling, tire modeling, engine
% modeling, and track simulation.
%
% Usage: Simply run this file - it contains everything needed for
%        complete lap time simulation including sample data.
%
% Features:
% - Pacejka '93 tire model implementation
% - Vehicle cornering, acceleration, and braking simulation
% - Engine force curve processing from CSV data
% - DRS (Drag Reduction System) functionality
% - Track segment-by-segment simulation
% - FSG points calculation
% - Comprehensive output and plotting
%
% !!READ USER GUIDE AND AGREEMENT BEFORE USE!!
%%

clear all; close all; clc;
warning off;
fprintf('LTS Coats 1.4 - Standalone Version\n');
fprintf('Complete Lap Time Simulation System\n');
fprintf('Loading all functionality...\n\n');

%% =========================== SETUP AND CONSTANTS ==========================

% Physical constants
g = 9.81;                    % Gravity (m/s^2)
rho = 1.204;                 % Air density (kg/m^3)

% Simulation parameters
dt = 0.001;                  % Time step (s)
roundoff = 3;               % Decimal places for rounding
v_tol = 0.001;              % Velocity convergence tolerance (m/s)
a_tol = 0.0001;             % Acceleration convergence tolerance (g)

% Velocity range for vehicle simulation
v_min = 5.25;               % Minimum velocity (m/s)
v_max = 35.0;               % Maximum velocity (m/s)
dv = 0.05;                  % Velocity step (m/s)
v_range = v_min:dv:v_max;   % Velocity range array

%% =========================== TIRE MODEL PARAMETERS ==========================

% Hoosier R25B 16x7.5x10 IA = 0deg P = 12psi - LATERAL
% 12 coefficient Pacejka '93 model
tire_coeff_lat = [1.497564443, -0.002272231, 2.840472825, 70.86957406, ...
                  0.222001058, 369690, -3.24E-07, 0.000447167, ...
                  -0.003834695, 0.002574449, -0.079271384, -6.855006239];

% Hoosier R25B 16x7.5x10 IA = 0deg P = 12psi - LONGITUDINAL
tire_coeff_lon = [1.2309, -0.0027, 2.9719, 1.1974, 0.0596, 46389, ...
                  -0.0140, 11.4756, 253.5530, 138.2941, 0.0003, -4.5750];

% Grip modification factors (friction multipliers)
mulat = 0.65;               % Lateral grip multiplier
mulon = 0.65;               % Longitudinal grip multiplier

% Slip limits for tire model
SA_max = 15;                % Maximum slip angle (deg)
SR_max = 0.25;              % Maximum slip ratio
SA_res = SA_max/50;         % Slip angle resolution (deg)
SR_res = SR_max/50;         % Slip ratio resolution
SA_range = 0:SA_res:SA_max; % Slip angle range array

% Store SA_range as persistent variable for use in functions
persistent SA_RANGE;
SA_RANGE = SA_range;

% Tire loaded radius
r_tire = 15.657/2 * 0.0254; % Tire radius (m)

%% =========================== VEHICLE PARAMETERS ===========================

% Vehicle masses (converted from lbs to kg)
m_driver = 180 * 0.453592;     % Driver mass (kg)
m_car = 280 * 0.453592;        % Car mass (kg)
m_accum = 60 * 0.453592;       % Accumulator mass (kg)
m_DRS = 1 * 0.453592;          % DRS mass (kg)
m_total = m_car + m_driver + m_accum + m_DRS; % Total mass (kg)

% Vehicle dimensions (converted from inches to meters)
track_width = 47 * 0.0254;     % Track width (m)
wheelbase = 60.25 * 0.0254;    % Wheelbase (m)
cg_h = 11.2 * 0.0254;          % CG height (m)
wdf = 0.445;                   % Weight distribution front (%)
finaldrive = 4;                % Final drive ratio

% Aerodynamic parameters
area = 1.15;                   % Frontal area (m^2)
Fl_base = 111 * 4.44822;       % Base downforce (N)
Fd_base = 45 * 4.44822;        % Base drag (N)
adf = 0.46;                    % Downforce fraction front (%)

% DRS parameters
drs_on = 1;                    % DRS enabled flag
drs_drag_reduction = 0.3;      % Drag reduction factor (0-1)
drs_downforce_loss = 0.1;      % Downforce loss factor (0-1)
g_thresh_g = 0.90;             % Lateral G threshold for DRS (g)
g_thresh = g_thresh_g * g;     % Lateral G threshold (m/s^2)

% Engine parameters
power_coeff = 1;               % Power scaling coefficient
shift_time = 0.1;              % Shift time (s)
rpm_limit = 5500;              % RPM limit

%% =========================== SAMPLE TRACK DATA ===========================
% This is sample track data for Ice Cream Cone 2 (simplified for testing)
% In a real implementation, these would be loaded from separate track files

% Track segment data:
% r = radius (m) - positive for left turns, negative for right turns, 0 for straights
% d = distance (m) for each segment
r = [0, -15, 0, 12, 0, -20, 0, 18, 0, -25, 0, 15, 0, -12, 0, 0];
d = [50, 25, 75, 30, 100, 35, 60, 25, 80, 40, 45, 30, 70, 20, 40, 120];

% Calculate total track distance
distance_total = sum(d);

fprintf('Track loaded: Sample Ice Cream Cone 2\n');
fprintf('Segments: %d, Total distance: %.1f m\n', numel(d), distance_total);

%% =========================== SAMPLE ENGINE DATA ===========================
% This is sample engine data for Emrax 208 HV motor
% In a real implementation, this would be loaded from CSV files

engine_data = [
    0,     0,     0,     0,  0;      % RPM, Power (hp), Torque (ft-lb), Fuel (L/hr), Gear ratio
    500,   10,    105,   0,  0;      % Sample data points
    1000,  25,    131,   0,  0;
    1500,  45,    157,   0,  0;
    2000,  70,    184,   0,  0;
    2500,  100,   210,   0,  0;
    3000,  135,   236,   0,  0;
    3500,  175,   262,   0,  0;
    4000,  220,   289,   0,  0;
    4500,  265,   309,   0,  0;
    5000,  300,   315,   0,  0;
    5500,  320,   305,   0,  0;
];

% Shifting speeds for each gear (m/s)
shifting_speeds = [10, 15, 20, 25, 30, 35, 40];

% Gear ratios (including primary)
gearing = [1, 3.5, 2.8, 2.2, 1.8, 1.5, 1.3, 1.1];

% Extract data
engine_spd = engine_data(:,1)';     % RPM
power = engine_data(:,2)';          % Power (hp)
torque = engine_data(:,3)';         % Torque (ft-lb)
fuel = engine_data(:,4)';           % Fuel flow (L/hr)
gearing_full = gearing;             % Gear ratios

% Convert units and apply power coefficient
power = power .* 0.745699872 .* power_coeff;    % Convert to kW
torque = torque .* 1.35582 .* power_coeff;      % Convert to N.m
fuel = fuel / 3600;                              % Convert to L/s

% Find number of gears
gearnum = numel(gearing_full) - 1;

% Store these as persistent variables for use in functions
persistent GEARNUM SHIFTING_SPEEDS;
GEARNUM = gearnum;
SHIFTING_SPEEDS = shifting_speeds;

%% =========================== TIRE MODEL FUNCTIONS ===========================

% Pacejka '93 tire model function
function Y = pacejka_fun_93(beta, in_vals)
    % in_vals = [Fz, X, mu] - normal force (N), slip value, friction coefficient
    Fz = in_vals(1);
    X = in_vals(2);
    mu = in_vals(3);

    % Extract tire coefficients
    C = beta(1); c1 = beta(2); c2 = beta(3); c3 = beta(4);
    c4 = beta(5); c5 = beta(6); c6 = beta(7); c7 = beta(8);
    c8 = beta(9); dE = beta(10); SH = beta(11); SV = beta(12);

    % Calculate tire model components
    D = mu .* (c1*Fz.^2 + c2*Fz);
    BCD = c3 * sind(c4 * atand(c5*Fz));
    B = BCD ./ (C .* D);
    x = X + SH;
    E = (c6*Fz.^2 + c7.*Fz + c8) + dE*sign(x);
    y = D .* sind(C .* atand(B.*x - E.*(B.*x - atand(B.*x))));

    % Add vertical shift and handle NaN values
    Y0 = y + SV;
    Y0(isnan(Y0)) = 0;
    Y = Y0;
end

% Calculate maximum lateral force for given normal force
function [max_Fy, alpha] = Fy_max(Fz, SA_max, res, tire_coeff_lat, mulat)
    tireFy = @(Fz, SA) pacejka_fun_93(tire_coeff_lat, [Fz, SA, mulat]);
    SA = 0:res:SA_max;

    for i = 1:length(SA)
        Fy(i) = tireFy(Fz, SA(i));
    end

    [max_Fy, I] = max(Fy);
    alpha = SA(I);
end

% Calculate lateral force array for given normal force and slip angle range
function Fy = Fy_range(Fz, SA_range, tire_coeff_lat, mulat)
    tireFy = @(Fz, SA) pacejka_fun_93(tire_coeff_lat, [Fz, SA, mulat]);

    for i = 1:length(SA_range)
        Fy(i) = tireFy(Fz, SA_range(i));
    end
end

% Calculate maximum driving force for given normal force
function max_Fx = Fx_drive(Fz, SR_max, res, tire_coeff_lon, mulon)
    tire = @(Fz, SR) pacejka_fun_93(tire_coeff_lon, [Fz, SR, mulon]);
    SR = 0:res:SR_max;

    for i = 1:length(SR)
        Fx(i) = tire(Fz, SR(i));
    end

    max_Fx = max(Fx);
end

% Calculate maximum braking force for given normal force
function max_Fx = Fx_brake(Fz, SR_max, res, tire_coeff_lon, mulon)
    tire = @(Fz, SR) pacejka_fun_93(tire_coeff_lon, [Fz, SR, mulon]);
    SR = 0:-res:-SR_max;

    for i = 1:length(SR)
        Fx(i) = tire(Fz, SR(i));
    end

    max_Fx = -min(Fx);
end

%% =========================== VEHICLE SIMULATION ===========================

% Main vehicle simulation function - calculates performance limits
function [Ay, Ax_drive, Ax_brake, alpha_out] = Vehicle_Sim_nlin(...
    v_range, m_total, track_width, wheelbase, cg_h, wdf, Fl, Fd, ...
    area, rho, tire_coeff_lat, tire_coeff_lon, mulat, mulon, ...
    SA_max, SA_res, SR_max, SR_res, finaldrive, Fx_engine, r_tire, a_tol)

    g = 9.81;

    % Initialize output arrays
    emptygrid = zeros(1, numel(v_range));
    Ay_out = emptygrid;
    Ax_drive_out = emptygrid;
    Ax_brake_out = emptygrid;
    alpha_out = zeros(numel(v_range), 4); % [LF, RF, LR, RR] slip angles

    % Create function handles for aerodynamic forces
    c_lift = 2*Fl/(rho*area*15.6464^2);
    c_drag = 2*Fd/(rho*area*15.6464^2);
    lift = @(v) 0.5*c_lift*rho*area.*v.^2;
    drag = @(v) 0.5*c_drag*rho*area.*v.^2;

    fprintf('  Generating vehicle model...');

    for i = 1:numel(v_range)
        % Aerodynamic forces at current velocity
        fdowns = lift(v_range(i)); % Downforce (N)
        fdrag = drag(v_range(i));  % Drag (N)

        % Static corner weights (N)
        w_1 = 0.5*m_total*g*wdf;
        w_2 = 0.5*m_total*g*wdf;
        w_3 = 0.5*m_total*g*(1-wdf);
        w_4 = 0.5*m_total*g*(1-wdf);

        % Calculate lateral acceleration limit (cornering)
        Ay_in = 0;
        Ay_last = 10;

        while abs(Ay_in - Ay_last) >= a_tol
            Ay_last = Ay_in;

            % Load transfer due to lateral acceleration
            LT_f = Ay_last*m_total*g*cg_h*wdf/track_width;
            LT_r = Ay_last*m_total*g*cg_h*(1-wdf)/track_width;

            % Cornering radii for each wheel
            R1 = (v_range(i)^2)/(Ay_last*g) - track_width/2;
            R2 = (v_range(i)^2)/(Ay_last*g) + track_width/2;
            R3 = (v_range(i)^2)/(Ay_last*g) - track_width/2;
            R4 = (v_range(i)^2)/(Ay_last*g) + track_width/2;

            % Steering angles
            heading1 = atand(wheelbase*(1-wdf)/R1);
            heading2 = atand(wheelbase*(1-wdf)/R2);
            heading3 = atand(wheelbase*wdf/R3);
            heading4 = atand(wheelbase*wdf/R4);

            % Slip angle differences
            SA_delta_f = heading1 - heading2;
            SA_delta_r = heading3 - heading4;

            % Normal forces at each wheel (convert to lbs for tire model)
            Fz_1 = (w_1 + 0.5*fdowns*0.5 - LT_f)/4.44822;
            Fz_2 = (w_2 + 0.5*fdowns*0.5 + LT_f)/4.44822;
            Fz_3 = (w_3 + 0.5*fdowns*0.5 - LT_r)/4.44822;
            Fz_4 = (w_4 + 0.5*fdowns*0.5 + LT_r)/4.44822;

            % Check for wheel lift-off
            if (Fz_1 + Fz_3) < 0
                error('Either CG is too high or Track is too narrow');
            end

            if Fz_1 < 0
                Fz_2 = Fz_2 - Fz_1;
                Fz_1 = 0;
            elseif Fz_3 < 0
                Fz_4 = Fz_4 - Fz_3;
                Fz_3 = 0;
            end

            % Calculate lateral forces and slip angles
            [Fy_1, alpha_1] = Fy_max(Fz_1, SA_max, SA_res, tire_coeff_lat, mulat);
            [Fy_2, alpha_2] = Fy_max(Fz_2, SA_max, SA_res, tire_coeff_lat, mulat);
            Fy_range_3 = Fy_range(Fz_3, SA_RANGE + SA_delta_r, tire_coeff_lat, mulat);
            Fy_range_4 = Fy_range(Fz_4, SA_RANGE, tire_coeff_lat, mulat);
            [Fy_rear, SA_r] = max(Fy_range_3 + Fy_range_4);
            Fy_3 = Fy_range_3(SA_r);
            Fy_4 = Fy_range_4(SA_r);
            alpha_3 = SA_RANGE(SA_r) + SA_delta_r;
            alpha_4 = SA_RANGE(SA_r);

            % Yaw moment balance
            Mz_cg_F = (Fy_1 + Fy_2) * (1 - wdf);
            Mz_cg_R = (Fy_3 + Fy_4) * wdf;

            if Mz_cg_F > Mz_cg_R
                Mz_cg_F = Mz_cg_R;
                Fy_F = Mz_cg_F / (1 - wdf);
                Fy_F_scale = Fy_F / (Fy_1 + Fy_2);
                Fy_R_scale = 1;
            elseif Mz_cg_F < Mz_cg_R
                Mz_cg_R = Mz_cg_F;
                Fy_R = Mz_cg_R / wdf;
                Fy_R_scale = Fy_R / (Fy_3 + Fy_4);
                Fy_F_scale = 1;
            else
                Fy_F_scale = 1;
                Fy_R_scale = 1;
            end

            % Calculate lateral acceleration
            Ay_in = (Fy_1*Fy_F_scale*4.44822 + Fy_2*Fy_F_scale*4.44822 + ...
                    Fy_3*Fy_R_scale*4.44822 + Fy_4*Fy_R_scale*4.44822) / (m_total * g);
        end

        alpha_out(i, :) = [alpha_1, alpha_2, alpha_3, alpha_4];
        Ay_out(i) = Ay_in * g;

        % Calculate longitudinal acceleration limit (driving)
        Ax_drive_in = 0;
        Ax_last = 10;

        while abs(Ax_drive_in - Ax_last) > a_tol
            Ax_last = Ax_drive_in;

            LT = g*m_total*Ax_last*cg_h/(wheelbase*2);

            Fz_1 = (w_1 + 0.5*fdowns*0.5 - LT)/4.44822;
            Fz_2 = (w_2 + 0.5*fdowns*0.5 - LT)/4.44822;
            Fz_3 = (w_3 + 0.5*fdowns*0.5 + LT)/4.44822;
            Fz_4 = (w_4 + 0.5*fdowns*0.5 + LT)/4.44822;

            if (Fz_1 + Fz_2) < 0
                error('Either CG is too far rearward or wheelbase is too short');
            end

            Fx_3 = Fx_drive(Fz_3, SR_max, SR_res, tire_coeff_lon, mulon);
            Fx_4 = Fx_drive(Fz_4, SR_max, SR_res, tire_coeff_lon, mulon);

            Fx_tire = Fx_3*4.44822 + Fx_4*4.44822;

            if Fx_tire >= Fx_engine(i)
                Ax_drive_in = (Fx_engine(i) - fdrag) / (m_total * g);
            else
                Ax_drive_in = (Fx_tire - fdrag) / (m_total * g);
            end
        end

        Ax_drive_out(i) = Ax_drive_in * g;

        % Calculate longitudinal acceleration limit (braking)
        Ax_brake_in = 0;
        Ax_last = 10;

        while abs(Ax_brake_in - Ax_last) > a_tol
            Ax_last = Ax_brake_in;

            LT = g*m_total*Ax_last*cg_h/(wheelbase*2);

            Fz_1 = (w_1 + 0.5*fdowns*0.5 - LT)/4.44822;
            Fz_2 = (w_2 + 0.5*fdowns*0.5 - LT)/4.44822;
            Fz_3 = (w_3 + 0.5*fdowns*0.5 + LT)/4.44822;
            Fz_4 = (w_4 + 0.5*fdowns*0.5 + LT)/4.44822;

            if (Fz_3 + Fz_4) < 0
                error('Either CG is too far forward or wheelbase is too short');
            end

            Fx_1 = -Fx_brake(Fz_1, SR_max, SR_res, tire_coeff_lon, mulon);
            Fx_2 = -Fx_brake(Fz_2, SR_max, SR_res, tire_coeff_lon, mulon);
            Fx_3 = -Fx_brake(Fz_3, SR_max, SR_res, tire_coeff_lon, mulon);
            Fx_4 = -Fx_brake(Fz_4, SR_max, SR_res, tire_coeff_lon, mulon);

            Fx_tire = Fx_1*4.44822 + Fx_2*4.44822 + Fx_3*4.44822 + Fx_4*4.44822;

            Ax_brake_in = (Fx_tire - fdrag) / (m_total * g);
        end

        Ax_brake_out(i) = Ax_brake_in * g;
    end

    % Fit polynomials to performance curves
    P1 = polyfit(v_range, Ay_out, 3);
    P2 = polyfit(v_range, Ax_drive_out, 10);
    P3 = polyfit(v_range, Ax_brake_out, 3);

    % Create function handles for performance curves
    Ay = @(x) P1(1).*x.^3 + P1(2).*x.^2 + P1(3).*x + P1(4);
    Ax_drive = @(x) P2(1).*x.^10 + P2(2).*x.^9 + P2(3).*x.^8 + P2(4).*x.^7 + ...
                   P2(5).*x.^6 + P2(6).*x.^5 + P2(7).*x.^4 + P2(8).*x.^3 + ...
                   P2(9).*x.^2 + P2(10).*x + P2(11);
    Ax_brake = @(x) P3(1).*x.^3 + P3(2).*x.^2 + P3(3).*x + P3(4);

    fprintf(' complete.\n');
end

%% =========================== ENGINE FORCE CURVES ===========================

% Process engine data and calculate force curves
function [Fx_engine, gear_pos, fuel_flow] = Engine_Force_Curves(...
    engine_spd, power, torque, fuel, gearing, finaldrive, ...
    v_range, r_tire, power_coeff, rpm_limit)

    % Find RPM tolerance for interpolation
    tol = engine_spd(1) - engine_spd(2);

    % Initialize arrays
    Fx_engine = zeros(1, length(v_range));
    gear_pos = zeros(1, length(v_range));
    fuel_flow = zeros(1, length(v_range));
    rpm = zeros(1, length(v_range));

    curr_gear = 1;
    primary = gearing(1);

    for ii = 1:length(v_range)
        v = v_range(ii);

        % Determine gear based on shifting speeds
        if GEARNUM == 1
            % Single gear case
            gear_pos(ii) = curr_gear;
            rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*...
                             (60/(2*pi))/r_tire), tol);
            ind = find(engine_spd <= rpm(ii));
            if ~isempty(ind)
                Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*...
                               finaldrive/r_tire;
            end
        elseif v < SHIFTING_SPEEDS(curr_gear)
            % Stay in current gear
            gear_pos(ii) = curr_gear;
            rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*...
                             (60/(2*pi))/r_tire), tol);
            ind = find(engine_spd <= rpm(ii));
            if ~isempty(ind)
                fuel_flow(ii) = fuel(ind(1));
                Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*...
                               finaldrive/r_tire;
            end
        elseif v > SHIFTING_SPEEDS(curr_gear) && v < SHIFTING_SPEEDS(end)
            % Shift up
            curr_gear = curr_gear + 1;
            gear_pos(ii) = curr_gear;
            rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*...
                             (60/(2*pi))/r_tire), tol);
            ind = find(engine_spd <= rpm(ii));
            if ~isempty(ind)
                fuel_flow(ii) = fuel(ind(1));
                Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*...
                               finaldrive/r_tire;
            end
        elseif v > SHIFTING_SPEEDS(end)
            % Maximum gear
            curr_gear = curr_gear + 1;
            gear_pos(ii) = curr_gear;
            rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*...
                             (60/(2*pi))/r_tire), tol);
            ind = find(engine_spd <= rpm(ii));
            if ~isempty(ind)
                fuel_flow(ii) = fuel(ind(1));
                Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*...
                               finaldrive/r_tire;
            end
            curr_gear = curr_gear - 1;
        end
    end

    % Apply RPM limit
    for ii = 1:length(v_range)
        if rpm(ii) > rpm_limit
            Fx_engine(ii) = 0;
        end
    end
end

% Round to nearest tolerance value
function y = round2(x, tol)
    y = round(x / tol) * tol;
end

%% =========================== TRACK SIMULATION ===========================

% Corner simulation function
function [v_, t_, throttle] = corner(Ay, r, d, roundoff, dt, t_start)
    % Calculate cornering velocity and time
    v = sqrt(Ay * r);  % Velocity for given lateral acceleration and radius
    t = d / v;         % Time to traverse segment
    t = round(t, roundoff);  % Round to specified precision

    t_end = t_start + t;
    t_ = t_start:dt:t_end;

    v_ = ones(1, numel(t_)) .* v;
    throttle = 0.3 .* ones(1, numel(t_)); % Partial throttle for cornering
end

% Corner simulation function (simple version)
function [v_, t_, throttle] = corner_simple(Ay, r, d, roundoff, dt, t_start)
    % Simplified corner simulation
    v = sqrt(Ay * r);
    t = d / v;
    t = round(t, roundoff);

    t_end = t_start + t;
    t_ = t_start:dt:t_end;

    v_ = ones(1, numel(t_)) .* v;
    throttle = 0.3 .* ones(1, numel(t_));
end

% Straight line simulation function
function [time, velocity, throttle] = Straight2(dist, dt, Ax_drive, Ax_brake, ...
                                               v_initial, v_final, vtol)
    % Simulate straight line acceleration/braking
    d = [dist];
    t_d(1) = 0;
    t_b(1) = 0;
    x_drive(1) = 0;
    x_brake(1) = 0;
    x_accd = 0;
    x_accb = 0;

    for i = 1:numel(d)
        j = 1;
        x_accd = 0;
        xd_drive_1(j) = v_initial;
        xdd_drive(j) = 0;

        % Acceleration phase
        while x_accd <= d(i)
            t_d(j+1) = t_d(j) + dt;
            xdd_drive(j+1) = Ax_drive(xd_drive_1(j));
            xd_drive_1(j+1) = xd_drive_1(j) + 0.5*(xdd_drive(j+1) + xdd_drive(j))*dt;
            x_drive(j+1) = x_drive(j) + 0.5*(xd_drive_1(j+1) + xd_drive_1(j))*dt;

            if i == 1
                x_accd = x_drive(j+1);
            else
                x_accd = x_drive(j+1) - sum(d(1:i-1));
            end
            j = j + 1;
        end

        % Braking phase
        k = 1;
        k_last = k;
        x_accb = d(i);
        t_b(k) = t_d(j-1);
        x_brake(k) = sum(d(1:i));
        xd_brake_1(k) = v_final;
        xdd_brake(k) = 0;

        while abs(x_brake(k)) <= d(i)
            t_b(k+1) = t_b(k) - dt;
            xdd_brake(k+1) = -Ax_brake(xd_brake_1(k));
            xd_brake_1(k+1) = xd_brake_1(k) + 0.5*(xdd_brake(k+1) + xdd_brake(k))*dt;
            x_brake(k+1) = x_brake(k) - 0.5*(xd_brake_1(k+1) + xd_brake_1(k))*dt;
            x_accb = x_accb + x_brake(k+1);
            k = k + 1;
        end
    end

    % Combine acceleration and braking data
    if numel(xd_drive_1) <= numel(xd_brake_1)
        ii_max = numel(xd_drive_1);
        xd_drive = xd_drive_1(1:ii_max);
        xd_brake = xd_brake_1(1:ii_max);
    else
        ii_max = numel(xd_brake_1);
        xd_drive = xd_drive_1(1:ii_max);
        xd_brake = xd_brake_1(1:ii_max);
    end

    xd_brake_flip = fliplr(xd_brake);

    % Determine throttle and velocity profiles
    throttle = zeros(1, ii_max);
    velocity = zeros(1, ii_max);
    jj = 1;
    time(1) = 0;

    for ii = 1:ii_max-1
        if xd_drive(ii) >= xd_brake_flip(ii)
            throttle(ii) = 0;  % Braking
            velocity(ii) = xd_brake_flip(ii);
        elseif xd_drive(ii) < xd_brake_flip(ii)
            throttle(ii) = 1;  % Driving
            velocity(ii) = xd_drive(ii);
        else
            error('Velocity calculation error');
        end

        if throttle(ii) ~= 1 && throttle(ii) ~= 0
            error('Throttle must be binary (0 or 1)');
        end

        time(jj+1) = time(jj) + dt;
        jj = jj + 1;
    end
end

% Main maneuver simulation function with DRS support
function [time_out, v_out, throttle_out, drs_active_for_segment, laptime] = ...
    Maneuver_Sim_fixed(r, d, Ay_on, Ay_off, Ax_drive_on, Ax_drive_off, ...
                       Ax_brake_on, Ax_brake_off, g_thresh, v_tol, ...
                       roundoff, dt, v_max, lift, r_tire, m_total, ...
                       cg_h, track_width, SA_max, SA_res)

    % Initialize output arrays
    time_out = zeros(numel(d), 1);
    v_out = zeros(numel(d), 1);
    throttle_out = zeros(numel(d), 1);
    drs_active_for_segment = zeros(numel(d), 1);

    % Initialize time
    t_start = 0;

    for i = 1:numel(d)
        if r(i) > 0  % Cornering segment
            % Determine if DRS should be active based on lateral G
            v_corner = sqrt(Ay_off(r(i)) * r(i));  % Use conservative estimate
            g_corner = v_corner^2 / (r(i) * 9.81);

            if g_corner < g_thresh && drs_on
                % Use DRS ON performance
                Ay = Ay_on;
                drs_active_for_segment(i) = 1;
            else
                % Use DRS OFF performance
                Ay = Ay_off;
                drs_active_for_segment(i) = 0;
            end

            % Corner simulation with appropriate performance model
            v_check = 0;
            corner_conv = 0;

            while corner_conv ~= 1
                [v_c, time_c, throttle_c] = corner(Ay(v_check), r(i), d(i), ...
                                                  roundoff, dt, t_start);
                v_i = mean(v_c);

                if abs(v_check - v_i) >= v_tol
                    v_check = abs(v_check + v_i) / 2;
                else
                    corner_conv = 1;
                end
            end

            % Store results
            for j = 1:numel(time_c)
                time_out(i, j) = time_c(j);
                v_out(i, j) = min(v_c(j), v_max);
                throttle_out(i, j) = throttle_c(j);
            end

            t_start = time_out(i, end);

        elseif r(i) == 0  % Straight segment
            % Straight simulation - DRS can be active on straights
            if drs_on
                % Use DRS ON performance for straights
                Ax_drive = Ax_drive_on;
                Ax_brake = Ax_brake_on;
                drs_active_for_segment(i) = 1;
            else
                % Use DRS OFF performance
                Ax_drive = Ax_drive_off;
                Ax_brake = Ax_brake_off;
                drs_active_for_segment(i) = 0;
            end

            % Calculate entry and exit velocities
            if i == 1  % Start of lap
                v_check = 0;
                corner_conv = 0;
                while corner_conv ~= 1
                    v_last = corner_simple(Ay_off(v_check), r(end), d(end), ...
                                          roundoff, dt, 0);
                    v_i = mean(v_last);
                    if abs(v_check - v_i) >= v_tol
                        v_check = abs(v_check + v_i) / 2;
                    else
                        corner_conv = 1;
                    end
                end
                v_exit = mean(v_last);

                v_check = 0;
                corner_conv = 0;
                while corner_conv ~= 1
                    v_next = corner_simple(Ay_off(v_check), r(i+1), d(i+1), ...
                                          roundoff, dt, 0);
                    v_i = mean(v_next);
                    if abs(v_check - v_i) >= v_tol
                        v_check = abs(v_check + v_i) / 2;
                    else
                        corner_conv = 1;
                    end
                end
                v_entry = mean(v_next);

            elseif i == numel(d)  % End of lap
                v_check = 0;
                corner_conv = 0;
                while corner_conv ~= 1
                    v_last = corner_simple(Ay_off(v_check), r(i-1), d(i-1), ...
                                          roundoff, dt, 0);
                    v_i = mean(v_last);
                    if abs(v_check - v_i) >= v_tol
                        v_check = abs(v_check + v_i) / 2;
                    else
                        corner_conv = 1;
                    end
                end
                v_exit = mean(v_last);

                v_check = 0;
                corner_conv = 0;
                while corner_conv ~= 1
                    v_next = corner_simple(Ay_off(v_check), r(1), d(1), ...
                                          roundoff, dt, 0);
                    v_i = mean(v_next);
                    if abs(v_check - v_i) >= v_tol
                        v_check = abs(v_check + v_i) / 2;
                    else
                        corner_conv = 1;
                    end
                end
                v_entry = mean(v_next);

            else  % Middle of lap
                v_check = 0;
                corner_conv = 0;
                while corner_conv ~= 1
                    v_last = corner_simple(Ay_off(v_check), r(i-1), d(i-1), ...
                                          roundoff, dt, 0);
                    v_i = mean(v_last);
                    if abs(v_check - v_i) >= v_tol
                        v_check = abs(v_check + v_i) / 2;
                    else
                        corner_conv = 1;
                    end
                end
                v_exit = mean(v_last);

                v_check = 0;
                corner_conv = 0;
                while corner_conv ~= 1
                    v_next = corner_simple(Ay_off(v_check), r(i+1), d(i+1), ...
                                          roundoff, dt, 0);
                    v_i = mean(v_next);
                    if abs(v_check - v_i) >= v_tol
                        v_check = abs(v_check + v_i) / 2;
                    else
                        corner_conv = 1;
                    end
                end
                v_entry = mean(v_next);
            end

            % Run straight simulation
            [time_s, v_s, throttle_s] = Straight2(d(i), dt, Ax_drive, Ax_brake, ...
                                                 v_exit, v_entry, v_tol);

            % Store results
            for k = 1:numel(time_s)
                time_out(i, k) = time_s(k);
                v_out(i, k) = v_s(k);
                throttle_out(i, k) = throttle_s(k);
            end

            t_start = time_out(i, end);
        end
    end

    % Calculate total lap time
    laptime = max(max(time_out));
end

%% =========================== MAIN SIMULATION ===========================

% Apply DRS settings to aerodynamic forces
if drs_on
    Fd = Fd_base * (1 - drs_drag_reduction);
    Fl = Fl_base * (1 - drs_downforce_loss);
    fprintf('DRS enabled: %.0f%% drag reduction, %.0f%% downforce loss\n', ...
            drs_drag_reduction*100, drs_downforce_loss*100);
else
    Fd = Fd_base;
    Fl = Fl_base;
    fprintf('DRS disabled\n');
end

% Precompute vehicle models for DRS ON and OFF
fprintf('Precomputing vehicle models...\n');

% DRS ON model
c_drag_on = 2*Fd/(rho*area*15.6464^2);
c_lift_on = 2*Fl/(rho*area*15.6464^2);
lift_on = @(v) 0.5*c_lift_on*rho*area.*v.^2;
drag_on = @(v) 0.5*c_drag_on*rho*area.*v.^2;

% Precompute vehicle models for DRS ON and OFF (without engine data initially)
[Ay_on, Ax_drive_on, Ax_brake_on, alpha_out_on] = Vehicle_Sim_nlin(...
    v_range, m_total, track_width, wheelbase, cg_h, wdf, Fl, Fd, ...
    area, rho, tire_coeff_lat, tire_coeff_lon, mulat, mulon, ...
    SA_max, SA_res, SR_max, SR_res, finaldrive, zeros(1, length(v_range)), ...
    r_tire, a_tol);

% DRS OFF model
Fd_off = Fd_base;
Fl_off = Fl_base;
c_drag_off = 2*Fd_off/(rho*area*15.6464^2);
c_lift_off = 2*Fl_off/(rho*area*15.6464^2);
lift_off = @(v) 0.5*c_lift_off*rho*area.*v.^2;
drag_off = @(v) 0.5*c_drag_off*rho*area.*v.^2;

[Ay_off, Ax_drive_off, Ax_brake_off, alpha_out_off] = Vehicle_Sim_nlin(...
    v_range, m_total, track_width, wheelbase, cg_h, wdf, Fl_off, Fd_off, ...
    area, rho, tire_coeff_lat, tire_coeff_lon, mulat, mulon, ...
    SA_max, SA_res, SR_max, SR_res, finaldrive, zeros(1, length(v_range)), ...
    r_tire, a_tol);

% Process engine data
fprintf('Processing engine data...\n');
[Fx_engine, gear_pos, fuel_flow] = Engine_Force_Curves(...
    engine_spd, power, torque, fuel, gearing, finaldrive, ...
    v_range, r_tire, power_coeff, rpm_limit);

% Run maneuver simulation
fprintf('Running lap simulation...\n');
[time_out, v_out, throttle_out, drs_active_for_segment, laptime] = ...
    Maneuver_Sim_fixed(r, d, Ay_on, Ay_off, Ax_drive_on, Ax_drive_off, ...
                      Ax_brake_on, Ax_brake_off, g_thresh, v_tol, ...
                      roundoff, dt, v_max, lift_on, r_tire, m_total, ...
                      cg_h, track_width, SA_max, SA_res);

% Display results
fprintf('\n=== SIMULATION RESULTS ===\n');
fprintf('Lap time: %.3f seconds\n', laptime);
fprintf('Average speed: %.2f m/s (%.1f km/h)\n', ...
        distance_total/laptime, (distance_total/laptime)*3.6);
fprintf('DRS active segments: %d/%d (%.1f%%)\n', ...
        sum(drs_active_for_segment), numel(d), ...
        sum(drs_active_for_segment)/numel(d)*100);

%% =========================== POST-PROCESSING ===========================

% Flatten outputs for plotting
time_vec = [];
speed_vec = [];
throttle_vec = [];
seg_idx_vec = [];
drs_map = [];

for i = 1:size(time_out, 1)
    nonz = find(time_out(i, :) ~= 0);
    time_vec = [time_vec time_out(i, nonz)];
    speed_vec = [speed_vec v_out(i, nonz)];
    throttle_vec = [throttle_vec throttle_out(i, nonz)];
    seg_idx_vec = [seg_idx_vec i*ones(1, numel(nonz))];
    drs_map = [drs_map drs_active_for_segment(i)*ones(1, numel(nonz))];
end

% Create plots
figure('Name', 'Speed vs Time');
scatter(time_vec, speed_vec, 6, throttle_vec, 'filled');
colorbar;
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Speed vs Time (throttle color)');
grid on;

figure('Name', 'DRS Activation');
bar(drs_active_for_segment);
xlabel('Segment');
ylabel('DRS Active (1=on)');
title('DRS Activation per Segment');
grid on;

figure('Name', 'Slip Angle Comparison');
plot(v_range, alpha_out_on(:, 1), '--', v_range, alpha_out_off(:, 1), '-');
legend('DRS ON', 'DRS OFF');
xlabel('Speed (m/s)');
ylabel('Slip Angle (deg)');
title('Front Left Slip Angle vs Speed');
grid on;

% Save simulation report
simReport.lap_time = laptime;
simReport.drs_active_segments = sum(drs_active_for_segment);
simReport.average_speed_ms = distance_total / laptime;
simReport.average_speed_kmh = (distance_total / laptime) * 3.6;
simReport.time_vec = time_vec;
simReport.speed_vec = speed_vec;
simReport.throttle_vec = throttle_vec;
simReport.drs_map = drs_map;

assignin('base', 'simReport', simReport);

fprintf('\nSimulation complete!\n');
fprintf('Results saved to workspace variable ''simReport''\n');
fprintf('Use simReport.lap_time to access the lap time\n');
fprintf('Use simReport.average_speed_kmh for average speed in km/h\n');

%% =========================== FSG POINTS CALCULATION ===========================

% FSG (Formula Student Germany) points calculation based on lap time
function points = calculate_fsg_points(laptime)
    % Points calculation based on 2018 FSG rules
    % This is a simplified version - actual rules may vary

    % Base times for different point levels (seconds)
    times = [75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150];
    points_table = [100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25];

    if laptime <= times(1)
        points = 100;  % Maximum points
    elseif laptime >= times(end)
        points = 25;   % Minimum points
    else
        % Interpolate between points
        for i = 1:length(times)-1
            if laptime >= times(i) && laptime <= times(i+1)
                ratio = (laptime - times(i)) / (times(i+1) - times(i));
                points = points_table(i) - ratio * (points_table(i) - points_table(i+1));
                break;
            end
        end
    end

    % Apply minimum time limit
    points = max(points, 25);
end

% Calculate FSG points
fsg_points = calculate_fsg_points(laptime);
fprintf('FSG Points: %.1f\n', fsg_points);

%% =========================== END OF SIMULATION ===========================

% Display completion message
fprintf('\nLTS Standalone simulation completed successfully!\n');
fprintf('Check the figures and simReport variable for detailed results.\n');
