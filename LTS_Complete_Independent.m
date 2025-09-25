%% LTS Complete Independent Simulation
% Comprehensive Formula Student/SAE vehicle lap time simulation
% Consolidates all functionalities into a single, standalone file
% Author: AI Assistant
% Date: 2024

function LTS_Complete_Independent()
    %% =========================== Setup =====================================
    warning off
    clc; 
    clear all; 
    close all;
    
    fprintf('LTS Complete Independent Simulation\n')
    fprintf('Formula Student Vehicle Lap Time Simulation\n\n')
    
    %% ====================== Simulation Parameters ==========================
    
    % Simulation Tolerance
    dt = 0.001; % Timestep
    roundoff = 3;   % Digits after decimal for time roundoff
    v_tol  = 0.001; % m/s Convergence tolerance for velocity
    a_tol = 0.0001; % g Convergence tolerance for acceleration
    
    %% ============================= Tire Model ===============================
    % 12 coeff Pacejka '93 Model Coefficients
    
    % Hoosier R25B 16x7.5x10 IA = 0deg P = 12psi
    tire_coeff_lat = [1.497564443, -0.002272231, 2.840472825, 70.86957406, 0.222001058, 369690, -3.24E-07, 0.000447167, -0.003834695, 0.002574449, -0.079271384, -6.855006239];
    tire_coeff_lon = [1.2309,-0.0027,2.9719,1.1974,0.0596,4.6389e+04,-0.0140,11.4756,253.5530,138.2941,0.0003,-4.5750];
    
    % Grip Modification Factor
    mulat = 0.65;
    mulon = 0.65;
    
    % Slip limits for tire model
    SA_max = 15; %deg
    SR_max = 0.25;
    SA_res = SA_max/50; %deg
    SR_res = SR_max/50;
    SA_range = 0:SA_res:SA_max;
    r_tire = 15.657/2 * 0.0254; % Tire loaded radius (m)
    
    % Velocity Range for vehicle sim
    v_min = 5.25;   % Lower velocity range for vehicle sim
    v_max = 35.0;   % Upper velocity range for vehicle sim
    dv = 0.05;      % Velocity differential for vehicle sim
    v_range = v_min:dv:v_max;
    
    %% ========================== Vehicle Setup ===============================
    
    % Chassis Setup
    m_driver = 180 * 0.453592;  % Driver weight (kg)
    m_car = 280 * 0.453592;     % Car weight (kg)
    m_accum = 60 * 0.453592;    % Accumulator weight (kg)
    m_DRS = 1 * 0.453592;       % DRS weight (kg)
    
    track = 47 * 0.0254;        % Track width (m)
    l = 60.25 * 0.0254;         % Wheelbase (m)
    cg_h = 11.2 * 0.0254;       % CG height (m)
    wdf = 0.445;                % Weight Distribution Front
    finaldrive = 4;             % Final drive ratio
    
    % Aerodynamics Setup
    adf = 0.46;                 % Downforce Fraction on Front (0-1)
    Fl_base = 111 * 4.44822;    % Base downforce @ 55 kph (N)
    Fd_base = 45 * 4.44822;     % Base drag @ 55 kph (N)
    area = 1.15;                % Frontal area (m^2)
    rho = 1.204;                % Air density (kg/m^3)
    
    % DRS Parameters
    drs_drag_reduction = 0.30;  % Drag reduction when DRS active
    drs_downforce_loss = 0.10;  % Downforce loss when DRS active
    drs_enabled = true;         % Enable DRS system
    
    % Engine Setup
    power_coeff = 1.0;          % Power scaling coefficient
    shift_time = 0.1;           % Shift time (s)
    rpm_limit = 5500;           % RPM limit
    
    % Calculate total mass
    m_total = m_car + m_driver + m_DRS + m_accum;
    a = l*(1-wdf);              % Distance from CG to rear axle
    b = l*wdf;                  % Distance from CG to front axle
    
    %% ========================== Create Sample Data =========================
    
    % Create sample track data (Ice Cream Cone track)
    [r, d, distance_total] = create_sample_track();
    enduro_laps = round(22000/sum(d));
    
    % Create sample engine data
    [engine_spd, power, torque, fuel, gearing, shifting] = create_sample_engine();
    
    %% ========================== Engine Force Curves ========================
    
    fprintf('Loading Engine Model...')
    
    % Convert units and apply scaling
    power = power .* 0.745699872 * power_coeff; % hp to kW
    torque = torque .* 1.35582 * power_coeff;   % ft.lbs to N.m
    shifting = shifting .* 0.44704;             % mph to m/s
    fuel = fuel/3600;                           % L/hr to L/s
    
    primary = gearing(1);
    gearnum = numel(gearing)-1;
    curr_gear = 1;
    tol = engine_spd(1)-engine_spd(2);
    
    % Calculate engine force at each velocity
    Fx_engine = zeros(size(v_range));
    fuel_flow = zeros(size(v_range));
    gear_pos = zeros(size(v_range));
    rpm = zeros(size(v_range));
    
    for ii = 1:length(v_range)
        v = v_range(ii);
        if gearnum == 1
            gear_pos(ii) = curr_gear;
            rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*(60/(2*pi))/r_tire),tol);
            ind = find(engine_spd <= rpm(ii));
            if ~isempty(ind)
                Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*finaldrive/r_tire;
                fuel_flow(ii) = fuel(ind(1));
            else
                Fx_engine(ii) = 0;
                fuel_flow(ii) = 0;
            end
        elseif curr_gear <= length(shifting) && v < shifting(curr_gear)
            gear_pos(ii) = curr_gear;
            rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*(60/(2*pi))/r_tire),tol);
            ind = find(engine_spd <= rpm(ii));
            if ~isempty(ind)
                fuel_flow(ii) = fuel(ind(1));
                Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*finaldrive/r_tire;
            else
                Fx_engine(ii) = 0;
                fuel_flow(ii) = 0;
            end
        elseif curr_gear < length(shifting) && v > shifting(curr_gear) && v < shifting(length(shifting))
            curr_gear = curr_gear + 1;
            gear_pos(ii) = curr_gear;
            rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*(60/(2*pi))/r_tire),tol);
            ind = find(engine_spd <= rpm(ii));
            if ~isempty(ind)
                fuel_flow(ii) = fuel(ind(1));
                Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*finaldrive/r_tire;
            else
                Fx_engine(ii) = 0;
                fuel_flow(ii) = 0;
            end
        elseif v > shifting(length(shifting))
            curr_gear = curr_gear + 1;
            gear_pos(ii) = curr_gear;
            rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*(60/(2*pi))/r_tire),tol);
            ind = find(engine_spd <= rpm(ii));
            if ~isempty(ind)
                fuel_flow(ii) = fuel(ind(1));
                Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*finaldrive/r_tire;
            else
                Fx_engine(ii) = 0;
                fuel_flow(ii) = 0;
            end
            curr_gear = curr_gear - 1;
        else
            % Default case
            gear_pos(ii) = curr_gear;
            rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*(60/(2*pi))/r_tire),tol);
            ind = find(engine_spd <= rpm(ii));
            if ~isempty(ind)
                fuel_flow(ii) = fuel(ind(1));
                Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*finaldrive/r_tire;
            else
                Fx_engine(ii) = 0;
                fuel_flow(ii) = 0;
            end
        end
    end
    
    % Apply RPM limit
    for ii = 1:length(v_range)
        if rpm(ii) > rpm_limit
            Fx_engine(ii) = 0;
        end
    end
    
    fprintf(' complete.\n')
    
    %% ========================== Vehicle Simulation =========================
    
    fprintf('Generating Vehicle Model...')
    
    % Precompute DRS ON and OFF configurations
    [Ay_on, Ax_drive_on, Ax_brake_on, alpha_out_on] = compute_vehicle_model(v_range, tire_coeff_lat, tire_coeff_lon, mulat, mulon, SA_max, SA_res, SR_max, SR_res, m_total, track, l, cg_h, wdf, adf, Fd_base*(1-drs_drag_reduction), Fl_base*(1-drs_downforce_loss), area, rho, Fx_engine, a_tol);
    
    [Ay_off, Ax_drive_off, Ax_brake_off, alpha_out_off] = compute_vehicle_model(v_range, tire_coeff_lat, tire_coeff_lon, mulat, mulon, SA_max, SA_res, SR_max, SR_res, m_total, track, l, cg_h, wdf, adf, Fd_base, Fl_base, area, rho, Fx_engine, a_tol);
    
    fprintf(' complete.\n')
    
    %% ========================== Simulation Runs ============================
    
    % Initialize results arrays
    i_max = 1; j_max = 1; run = 0;
    
    for iii = 1:i_max
        for jjj = 1:j_max
            run = run + 1;
            fprintf('-------------------------------------\n')
            fprintf('Simulating Setup %2.0f/%2.0f\n',run,i_max*j_max)
            
            % Clear previous results
            clear time_out v_out throttle_out time_vec v_vec throttle_vec time speed throttle
            
            %% ========================== Acceleration Simulation =============
            fprintf('Simulating Accel... ')
            acceltime = 0;
            w = 1;
            d_accel(1) = 0;
            xd_accel(1) = 5; % Initial velocity
            xdd_accel(1) = 0;
            
            while d_accel(w) <= 76 % 75m acceleration test
                acceltime = acceltime + dt;
                xdd_accel(w+1) = abs(Ax_drive_on(xd_accel(w)));
                xd_accel(w+1) = xd_accel(w) + 0.5*dt*(xdd_accel(w+1)+xdd_accel(w));
                d_accel(w+1) = d_accel(w) + 0.5*dt*(xd_accel(w+1)+xd_accel(w));
                w = w+1;
            end
            
            acceltime = acceltime + shift_time*numel(shifting);
            fprintf('complete.\n')
            
            %% ========================== Skidpad Simulation ==================
            fprintf('Simulating Skidpad... ')
            v_check = 0;
            corner_conv = 0;
            r_skid = 7.625+track/2;
            
            while corner_conv ~= 1
                [v_skid, time_skid] = corner(Ay_on(v_check), r_skid, r_skid*2*pi, roundoff, dt, 0);
                v_i = mean(v_skid);
                if abs(v_check-v_i) >= v_tol
                    v_check = abs(v_check+v_i)/2;
                else
                    corner_conv = 1;
                end
            end
            
            skidpadtime = max(time_skid);
            fprintf('complete.\n')
            
            %% ========================== Lap Simulation ======================
            fprintf('Simulating Lap...')
            
            % Initialize output arrays
            time_out = zeros(numel(d), 20000);
            v_out = zeros(numel(d), 20000);
            throttle_out = zeros(numel(d), 20000);
            drs_active_for_segment = ones(1, numel(d));
            
            for i = 1:numel(d)
                if r(i) > 0 % Cornering case
                    % Try DRS ON first
                    Ay_try = Ay_on;
                    v_check = 0;
                    corner_conv = 0;
                    
                    while corner_conv ~= 1
                        [v_c, time_c, throttle_c] = corner(Ay_try(v_check), r(i), d(i), roundoff, dt, 0);
                        v_i = mean(v_c);
                        if abs(v_check-v_i) >= v_tol
                            v_check = abs(v_check+v_i)/2;
                        else
                            corner_conv = 1;
                        end
                    end
                    
                    % Check if DRS ON can handle the corner
                    ay_actual = (v_i^2)/r(i);
                    if ay_actual > 0.9*9.81 % If lateral G > 0.9g, use DRS OFF
                        Ay_try = Ay_off;
                        v_check = 0;
                        corner_conv = 0;
                        
                        while corner_conv ~= 1
                            [v_c, time_c, throttle_c] = corner(Ay_try(v_check), r(i), d(i), roundoff, dt, 0);
                            v_i = mean(v_c);
                            if abs(v_check-v_i) >= v_tol
                                v_check = abs(v_check+v_i)/2;
                            else
                                corner_conv = 1;
                            end
                        end
                        drs_active_for_segment(i) = 0;
                    end
                    
                    % Store corner results
                    for j = 1:numel(time_c)
                        time_out(i,j) = time_c(j);
                        if v_c(j) >= v_max
                            v_out(i,j) = v_max;
                        else
                            v_out(i,j) = v_c(j);
                        end
                        throttle_out(i,j) = throttle_c(j);
                    end
                    
                elseif r(i) == 0 % Straight case
                    % Determine entry and exit velocities
                    if i == 1
                        if r(numel(d)) > 0
                            v_exit = mean(corner_simple(Ay_on(0), r(numel(d)), d(numel(d)), roundoff, dt, 0));
                        else
                            v_exit = 10; % Default exit velocity for straight
                        end
                        if r(i+1) > 0
                            v_entry = mean(corner_simple(Ay_on(0), r(i+1), d(i+1), roundoff, dt, 0));
                        else
                            v_entry = 10; % Default entry velocity for straight
                        end
                    elseif i == numel(d)
                        if r(i-1) > 0
                            v_exit = mean(corner_simple(Ay_on(0), r(i-1), d(i-1), roundoff, dt, 0));
                        else
                            v_exit = 10;
                        end
                        if r(1) > 0
                            v_entry = mean(corner_simple(Ay_on(0), r(1), d(1), roundoff, dt, 0));
                        else
                            v_entry = 10;
                        end
                    else
                        if r(i-1) > 0
                            v_exit = mean(corner_simple(Ay_on(0), r(i-1), d(i-1), roundoff, dt, 0));
                        else
                            v_exit = 10;
                        end
                        if r(i+1) > 0
                            v_entry = mean(corner_simple(Ay_on(0), r(i+1), d(i+1), roundoff, dt, 0));
                        else
                            v_entry = 10;
                        end
                    end
                    
                    % Use DRS ON for straights
                    [time_s, v_s, throttle_s] = Straight2(d(i), dt, Ax_drive_on, Ax_brake_on, v_exit, v_entry, v_tol);
                    
                    % Store straight results
                    for k = 1:numel(time_s)
                        time_out(i,k) = time_s(k);
                        v_out(i,k) = v_s(k);
                        throttle_out(i,k) = throttle_s(k);
                    end
                end
            end
            
            fprintf(' complete.\n')
            
            %% ========================== Post Processing =====================
            fprintf('Post Processing...')
            
            % Convert matrices to vectors
            limits = size(time_out);
            time_vec = [];
            v_vec = [];
            throttle_vec = [];
            ref = 0;
            
            for q = 1:limits(1)
                for qq = 1:limits(2)
                    if time_out(q,qq) ~= 0
                        time_vec(end+1) = time_out(q,qq) + ref;
                        v_vec(end+1) = v_out(q,qq);
                        throttle_vec(end+1) = throttle_out(q,qq);
                    end
                end
                ref = max(time_out(q,:)) + ref;
            end
            
            % Clean up data
            zz = 0;
            time = [];
            speed = [];
            throttle = [];
            
            for z = 1:numel(time_vec)
                if time_vec(z) ~= 0
                    zz = zz+1;
                    time(zz) = time_vec(z);
                    speed(zz) = v_vec(z);
                    if speed(zz) > v_max
                        speed(zz) = v_max;
                    end
                    throttle(zz) = throttle_vec(z);
                end
            end
            
            if zz > 1
                speed(1) = speed(2);
            elseif zz == 0
                % Handle case where no valid data was found
                time = [0, 1];
                speed = [5, 5];
                throttle = [0, 0];
                zz = 2;
            end
            
            laptime = max(time);
            endurotime = laptime * enduro_laps;
            fuelused_adj = 0; % Simplified for this example
            
            fprintf(' complete. \n\n')
        end
    end
    
    %% ========================== Point Calculation ==========================
    
    tbest = laptime;
    endurobest = tbest * enduro_laps;
    skidbest = skidpadtime;
    accelbest = acceltime;
    fuelbest = fuelused_adj;
    
    % Calculate points (simplified FSG scoring)
    autox_pts = 95.5*((((tbest*1.25)/laptime)-1)/0.25) + 4.5;
    enduro_pts = 300*((((endurobest*1.333)/endurotime)-1)/0.333) + 25;
    skidpad_pts = 71.5*((((skidbest*1.25)/skidpadtime)^2-1)/0.5625) + 3.5;
    accel_pts = 71.5*(((accelbest*1.5/acceltime)-1)/0.5) + 3.5;
    total_pts = autox_pts + enduro_pts + skidpad_pts + accel_pts;
    
    %% ========================== Results Output =============================
    
    avg_vel = sum(speed.*dt/max(time));
    
    % Print Results
    fprintf('=== SIMULATION RESULTS ===\n')
    fprintf('Autocross Laptime = %5.3f s\n', laptime)
    fprintf('Endurance Time = \t%5.3f s\n', endurotime)
    fprintf('Skidpad Time = \t\t%5.3f s\n', skidpadtime)
    fprintf('Accel Time = \t\t%5.3f s\n', acceltime)
    fprintf('Average Velocity = \t%5.3f m/s\n', avg_vel)
    fprintf('Total Points = \t\t%5.1f pts\n\n', total_pts)
    
    %% ========================== Plotting ===================================
    
    % Time vs velocity plot
    figure('Name', 'LTS Simulation Results', 'Position', [100, 100, 1200, 800])
    
    subplot(2,3,1)
    scatter(time, speed, 2, throttle, 'o')
    grid on
    colormap jet
    title('Time vs Speed')
    xlabel('Time (s)')
    ylabel('Speed (m/s)')
    colorbar
    
    % Slip angle vs Velocity
    subplot(2,3,2)
    hold on
    if exist('alpha_out_on', 'var') && size(alpha_out_on, 2) >= 2
        plot(v_range, alpha_out_on(:,1), '--', 'DisplayName', 'LF DRS ON')
        plot(v_range, alpha_out_on(:,2), '--', 'DisplayName', 'RF DRS ON')
    end
    if exist('alpha_out_off', 'var') && size(alpha_out_off, 2) >= 2
        plot(v_range, alpha_out_off(:,1), '-', 'DisplayName', 'LF DRS OFF')
        plot(v_range, alpha_out_off(:,2), '-', 'DisplayName', 'RF DRS OFF')
    end
    legend('Location', 'best')
    title('Slip angle vs Speed')
    xlabel('Speed (m/s)')
    ylabel('Slip Angle (degrees)')
    grid on
    
    % DRS activation
    subplot(2,3,3)
    bar(drs_active_for_segment)
    title('DRS Activation per Segment')
    xlabel('Segment')
    ylabel('DRS Active (1=on)')
    grid on
    
    % Engine force curve
    subplot(2,3,4)
    plot(v_range, Fx_engine)
    title('Engine Force vs Speed')
    xlabel('Speed (m/s)')
    ylabel('Force (N)')
    grid on
    
    % Vehicle performance curves
    subplot(2,3,5)
    hold on
    if exist('Ay_on', 'var')
        plot(v_range, Ay_on(v_range), '--', 'DisplayName', 'Lateral G DRS ON')
    end
    if exist('Ay_off', 'var')
        plot(v_range, Ay_off(v_range), '-', 'DisplayName', 'Lateral G DRS OFF')
    end
    if exist('Ax_drive_on', 'var')
        plot(v_range, Ax_drive_on(v_range), '--', 'DisplayName', 'Long G Drive DRS ON')
    end
    if exist('Ax_brake_on', 'var')
        plot(v_range, Ax_brake_on(v_range), '--', 'DisplayName', 'Long G Brake DRS ON')
    end
    legend('Location', 'best')
    title('Vehicle Performance Curves')
    xlabel('Speed (m/s)')
    ylabel('Acceleration (m/s²)')
    grid on
    
    % Gear position
    subplot(2,3,6)
    plot(v_range, gear_pos)
    title('Gear Position vs Speed')
    xlabel('Speed (m/s)')
    ylabel('Gear')
    grid on
    
    fprintf('Simulation complete! Check the generated plots for results.\n')
    
end

%% ========================== Helper Functions ==============================

function [r, d, distance_total] = create_sample_track()
    % Create a sample Ice Cream Cone track
    % This is a simplified track with alternating straights and corners
    
    % Track segments: [radius, distance]
    % radius = 0 for straights, > 0 for corners
    segments = [
        0, 50;      % Straight 1
        30, 47.1;   % Corner 1 (90 deg)
        0, 100;     % Straight 2
        25, 39.3;   % Corner 2 (90 deg)
        0, 80;      % Straight 3
        35, 54.9;   % Corner 3 (90 deg)
        0, 60;      % Straight 4
        20, 31.4;   % Corner 4 (90 deg)
        0, 40;      % Straight 5
    ];
    
    r = segments(:,1)';
    d = segments(:,2)';
    distance_total = sum(d);
end

function [engine_spd, power, torque, fuel, gearing, shifting] = create_sample_engine()
    % Create sample Emrax 208 HV motor data
    
    % RPM range
    engine_spd = 0:100:6000;
    
    % Power curve (kW) - typical electric motor curve
    power = zeros(size(engine_spd));
    for i = 1:length(engine_spd)
        if engine_spd(i) <= 3000
            power(i) = 80 * (engine_spd(i)/3000);
        else
            power(i) = 80 * (1 - 0.3*(engine_spd(i)-3000)/3000);
        end
    end
    
    % Torque curve (N.m) - constant torque up to base speed
    torque = zeros(size(engine_spd));
    for i = 1:length(engine_spd)
        if engine_spd(i) <= 3000
            torque(i) = 255; % Constant torque
        else
            torque(i) = 255 * (3000/engine_spd(i));
        end
    end
    
    % Fuel consumption (L/hr) - simplified for electric
    fuel = 0.1 * ones(size(engine_spd));
    
    % Gearing (primary, gear1, gear2, etc.)
    gearing = [1.0, 2.5, 1.8, 1.3, 1.0];
    
    % Shifting speeds (mph)
    shifting = [0, 15, 25, 35];
end

function [Ay, Ax_drive, Ax_brake, alpha_out] = compute_vehicle_model(v_range, tire_coeff_lat, tire_coeff_lon, mulat, mulon, SA_max, SA_res, SR_max, SR_res, m_total, track, l, cg_h, wdf, adf, Fd, Fl, area, rho, Fx_engine, a_tol)
    % Compute vehicle model for given aerodynamic configuration
    
    g = 9.81;
    emptygrid = zeros(1, numel(v_range));
    Ay_out = emptygrid;
    Ax_drive_out = emptygrid;
    Ax_brake_out = emptygrid;
    alpha_out = zeros(numel(v_range), 4);
    
    % Convert forces to coefficients
    c_lift = 2*Fl/(rho*area*15.6464^2);
    c_drag = 2*Fd/(rho*area*15.6464^2);
    lift = @(v) 0.5*c_lift*rho*area*v^2;
    drag = @(v) 0.5*c_drag*rho*area*v^2;
    
    for i = 1:numel(v_range)
        fdowns = lift(v_range(i));
        fdrag = drag(v_range(i));
        
        % Lateral limit calculation
        Ay_in = 0;
        Ay_last = 10;
        
        while abs(Ay_in - Ay_last) >= a_tol
            Ay_last = Ay_in;
            
            LT_f = Ay_last*m_total*g*cg_h*wdf/track;
            LT_r = Ay_last*m_total*g*cg_h*(1-wdf)/track;
            
            R1 = (v_range(i)^2)/(Ay_last*g) - track/2;
            R2 = (v_range(i)^2)/(Ay_last*g) + track/2;
            R3 = (v_range(i)^2)/(Ay_last*g) - track/2;
            R4 = (v_range(i)^2)/(Ay_last*g) + track/2;
            
            heading1 = atand(l*(1-wdf)/(R1));
            heading2 = atand(l*(1-wdf)/(R2));
            heading3 = atand(l*(wdf)/(R3));
            heading4 = atand(l*(wdf)/(R4));
            
            SA_delta_f = heading1 - heading2;
            SA_delta_r = heading3 - heading4;
            
            % Static corner weights
            w_1 = 0.5*m_total*g*wdf;
            w_2 = 0.5*m_total*g*wdf;
            w_3 = 0.5*m_total*g*(1-wdf);
            w_4 = 0.5*m_total*g*(1-wdf);
            
            Fz_1 = (w_1 + 0.5*fdowns*adf - LT_f)/4.44822;
            Fz_2 = (w_2 + 0.5*fdowns*adf + LT_f)/4.44822;
            Fz_3 = (w_3 + 0.5*fdowns*(1-adf) - LT_r)/4.44822;
            Fz_4 = (w_4 + 0.5*fdowns*(1-adf) + LT_r)/4.44822;
            
            % Ensure positive normal forces
            if Fz_1 < 0
                Fz_2 = Fz_2 - Fz_1;
                Fz_1 = 0;
            end
            if Fz_3 < 0
                Fz_4 = Fz_4 - Fz_3;
                Fz_3 = 0;
            end
            
            % Calculate tire forces
            [Fy_1, alpha_1] = Fy_max(Fz_1, SA_max, SA_res, tire_coeff_lat, mulat);
            [Fy_2, alpha_2] = Fy_max(Fz_2, SA_max, SA_res, tire_coeff_lat, mulat);
            Fy_range_3 = Fy_range(Fz_3, SA_range+SA_delta_r, tire_coeff_lat, mulat);
            Fy_range_4 = Fy_range(Fz_4, SA_range, tire_coeff_lat, mulat);
            [Fy_rear, SA_r] = max(Fy_range_3+Fy_range_4);
            Fy_3 = Fy_range_3(SA_r);
            Fy_4 = Fy_range_4(SA_r);
            alpha_3 = SA_range(SA_r)+SA_delta_r;
            alpha_4 = SA_range(SA_r);
            
            % Balance front and rear moments
            Mz_cg_F = (Fy_1+Fy_2)*(1-wdf);
            Mz_cg_R = (Fy_3+Fy_4)*(wdf);
            
            if Mz_cg_F > Mz_cg_R
                Mz_cg_F = Mz_cg_R;
                Fy_F = Mz_cg_F/(1-wdf);
                Fy_F_scale = Fy_F/(Fy_1 + Fy_2);
                Fy_R_scale = 1;
            elseif Mz_cg_F < Mz_cg_R
                Mz_cg_R = Mz_cg_F;
                Fy_R = Mz_cg_R/(wdf);
                Fy_R_scale = Fy_R/(Fy_3 + Fy_4);
                Fy_F_scale = 1;
            else
                Fy_F_scale = 1;
                Fy_R_scale = 1;
            end
            
            Ay_in = (Fy_1*Fy_F_scale*4.44822+Fy_2*Fy_F_scale*4.44822+Fy_3*Fy_R_scale*4.44822+Fy_4*Fy_R_scale*4.44822)/(m_total*g);
        end
        
        alpha_out(i,:) = [alpha_1 alpha_2 alpha_3 alpha_4];
        Ay_out(i) = Ay_in*g;
        
        % Driving limit
        Ax_drive_in = 0;
        Ax_last = 10;
        
        while abs(Ax_drive_in-Ax_last) > a_tol
            Ax_last = Ax_drive_in;
            
            LT = g*m_total*Ax_last*cg_h/(l*2);
            
            Fz_1 = (w_1 + 0.5*fdowns*adf - LT)/4.44822;
            Fz_2 = (w_2 + 0.5*fdowns*adf - LT)/4.44822;
            Fz_3 = (w_3 + 0.5*fdowns*(1-adf) + LT)/4.44822;
            Fz_4 = (w_4 + 0.5*fdowns*(1-adf) + LT)/4.44822;
            
            Fx_3 = Fx_drive(Fz_3, SR_max, SR_res, tire_coeff_lon, mulon);
            Fx_4 = Fx_drive(Fz_4, SR_max, SR_res, tire_coeff_lon, mulon);
            
            Fx_tire = Fx_3*4.44822+Fx_4*4.44822;
            
            if Fx_tire >= Fx_engine(i)
                Ax_drive_in = (Fx_engine(i) - fdrag)/(m_total*g);
            else
                Ax_drive_in = (Fx_tire - fdrag)/(m_total*g);
            end
        end
        
        Ax_drive_out(i) = Ax_drive_in*g;
        
        % Braking limit
        Ax_brake_in = 0;
        Ax_last = 10;
        
        while abs(Ax_brake_in-Ax_last) > a_tol
            Ax_last = Ax_brake_in;
            
            LT = g*m_total*Ax_last*cg_h/(l*2);
            
            Fz_1 = (w_1 + 0.5*fdowns*adf - LT)/4.44822;
            Fz_2 = (w_2 + 0.5*fdowns*adf - LT)/4.44822;
            Fz_3 = (w_3 + 0.5*fdowns*(1-adf) + LT)/4.44822;
            Fz_4 = (w_4 + 0.5*fdowns*(1-adf) + LT)/4.44822;
            
            Fx_1 = -Fx_brake(Fz_1, SR_max, SR_res, tire_coeff_lon, mulon);
            Fx_2 = -Fx_brake(Fz_2, SR_max, SR_res, tire_coeff_lon, mulon);
            Fx_3 = -Fx_brake(Fz_3, SR_max, SR_res, tire_coeff_lon, mulon);
            Fx_4 = -Fx_brake(Fz_4, SR_max, SR_res, tire_coeff_lon, mulon);
            
            Fx_tire = Fx_1*4.44822+Fx_2*4.44822+Fx_3*4.44822+Fx_4*4.44822;
            
            Ax_brake_in = (Fx_tire-fdrag)/(m_total*g);
        end
        
        Ax_brake_out(i) = Ax_brake_in*g;
    end
    
    % Create polynomial fits
    if length(v_range) >= 4
        P1 = polyfit(v_range, Ay_out, min(3, length(v_range)-1));
        P3 = polyfit(v_range, Ax_brake_out, min(3, length(v_range)-1));
    else
        P1 = polyfit(v_range, Ay_out, 1);
        P3 = polyfit(v_range, Ax_brake_out, 1);
    end
    
    if length(v_range) >= 11
        P2 = polyfit(v_range, Ax_drive_out, 10);
    else
        P2 = polyfit(v_range, Ax_drive_out, min(5, length(v_range)-1));
    end
    
    Ay = @(x) polyval(P1, x);
    Ax_drive = @(x) polyval(P2, x);
    Ax_brake = @(x) polyval(P3, x);
end

function [max_Fx] = Fx_drive(Fz, SR_max, res, tire_coeff_lon, mulon)
    tire = @(Fz,SR) pacejka_fun_93(tire_coeff_lon, [Fz SR mulon]);
    
    SR = 0:res:SR_max;
    Fx = zeros(size(SR));
    for i = 1:length(SR)
        Fx(i) = tire(Fz, SR(i));
    end
    max_Fx = max(Fx);
end

function [max_Fx] = Fx_brake(Fz, SR_max, res, tire_coeff_lon, mulon)
    tire = @(Fz,SR) pacejka_fun_93(tire_coeff_lon, [Fz SR mulon]);
    
    SR = 0:-res:-SR_max;
    Fx = zeros(size(SR));
    for i = 1:length(SR)
        Fx(i) = tire(Fz, SR(i));
    end
    max_Fx = -min(Fx);
end

function [max_Fy, alpha] = Fy_max(Fz, SA_max, res, tire_coeff_lat, mulat)
    tireFy = @(Fz,SA) pacejka_fun_93(tire_coeff_lat, [Fz SA mulat]);
    
    SA = 0:res:SA_max;
    Fy = zeros(size(SA));
    for i = 1:length(SA)
        Fy(i) = tireFy(Fz, SA(i));
    end
    [max_Fy, I] = max(Fy);
    alpha = SA(I);
end

function [Fy] = Fy_range(Fz, SA_range, tire_coeff_lat, mulat)
    tireFy = @(Fz,SA) pacejka_fun_93(tire_coeff_lat, [Fz SA mulat]);
    
    Fy = zeros(size(SA_range));
    for i = 1:length(SA_range)
        Fy(i) = tireFy(Fz, SA_range(i));
    end
end

function Y = pacejka_fun_93(beta, in)
    % Pacejka '93 tire model
    Fz = in(1);
    X = in(2);
    mu = in(3);
    
    % beta = [C,c1,c2,c3,c4,c5,c6,c7,c8,dE,SH,SV]
    C = beta(1);
    c1 = beta(2);
    c2 = beta(3);
    c3 = beta(4);
    c4 = beta(5);
    c5 = beta(6);
    c6 = beta(7);
    c7 = beta(8);
    c8 = beta(9);
    dE = beta(10);
    SH = beta(11);
    SV = beta(12);
    
    D = mu.*(c1*Fz.^2+c2*Fz);
    BCD = c3*sind(c4*atand(c5*Fz));
    B = BCD./(C.*D);
    x = X + SH;
    E = (c6*Fz.^2+c7.*Fz+c8)+dE*sign(x);
    y = D.*sind(C.*atand(B.*x-E.*(B.*x-atand(B.*x))));
    Y0 = y + SV;
    Y0(isnan(Y0)) = 0;
    Y = Y0;
end

function [v_, t_, throttle] = corner(Ay, r, d, roundoff, dt, t_start)
    % Calculates cornering velocity and time
    v = sqrt(Ay*r);
    t = d/v;
    t = round(t, roundoff);
    
    t_end = t_start + t;
    t_ = t_start:dt:t_end;
    
    v_ = ones(1, numel(t_)).*v;
    throttle = 0.3.*ones(1, numel(t_));
end

function [v_] = corner_simple(Ay, r, d, roundoff, dt, t_start)
    % Simplified corner calculation (returns only velocity)
    v = sqrt(Ay*r);
    t = d/v;
    t = round(t, roundoff);
    
    t_end = t_start + t;
    t_ = t_start:dt:t_end;
    
    v_ = ones(1, numel(t_)).*v;
end

function [time, velocity, throttle] = Straight2(dist, dt, Ax_drive, Ax_brake, v_initial, v_final, vtol)
    % Simulates straight segment with acceleration and braking
    
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
        
        while x_accd <= d(i)
            t_d(j+1) = t_d(j) + dt;
            xdd_drive(j+1) = Ax_drive(xd_drive_1(j));
            xd_drive_1(j+1) = xd_drive_1(j) + 0.5*(xdd_drive(j+1)+xdd_drive(j))*dt;
            x_drive(j+1) = x_drive(j) + 0.5*(xd_drive_1(j+1) + xd_drive_1(j))*dt;
            if i == 1
                x_accd = x_drive(j+1);
            else
                x_accd = x_drive(j+1) - sum(d(1:i-1));
            end
            j = j+1;
        end
        
        k = 1;
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
            k = k+1;
        end
    end
    
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
    
    throttle = zeros(1, ii_max);
    velocity = zeros(1, ii_max);
    jj = 1;
    time(1) = 0;
    
    for ii = 1:ii_max-1
        if xd_drive(ii) >= xd_brake_flip(ii)
            throttle(ii) = 0;
            velocity(ii) = xd_brake_flip(ii);
        else
            throttle(ii) = 1;
            velocity(ii) = xd_drive(ii);
        end
        
        time(jj+1) = time(jj) + dt;
        jj = jj+1;
    end
end

function z = round2(x, y)
    % Round number to nearest multiple of arbitrary precision
    if nargin ~= 2
        error('round2 requires exactly 2 arguments');
    end
    if numel(y) > 1
        error('Y must be scalar');
    end
    z = round(x/y)*y;
end