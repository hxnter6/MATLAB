%% LTS_Consolidated_Complete.m
% =========================================================================
% Lap Time Simulation - Consolidated Complete Version
% =========================================================================
% This is a fully independent, consolidated version of the LTS lap time 
% simulation with all functionalities in a single file. It includes:
%   - Vehicle dynamics modeling
%   - Tire modeling (Pacejka '93)
%   - Aerodynamics with DRS support
%   - Track simulation (corners and straights)
%   - Performance metrics calculation
%   - Visualization and plotting
%
% Author: Consolidated from original LTS code
% Date: Created from multiple source files
% =========================================================================

%% MAIN EXECUTION FUNCTION
function LTS_Consolidated_Complete()
    % Clear workspace and set up environment
    clearvars -except simReport; 
    close all; 
    clc;
    warning off;
    
    fprintf('=====================================\n');
    fprintf('LTS Consolidated Complete Version\n');
    fprintf('Lap Time Simulation System\n');
    fprintf('=====================================\n\n');
    
    % Initialize global variables
    initializeGlobalVariables();
    
    % Set up simulation parameters
    params = setupSimulationParameters();
    
    % Create or load track data
    [track_data, params] = setupTrackData(params);
    
    % Set up vehicle parameters
    vehicle = setupVehicleParameters(params);
    
    % Set up engine/motor data
    engine = setupEngineData(vehicle, params);
    
    % Run the main simulation
    results = runMainSimulation(track_data, vehicle, engine, params);
    
    % Calculate performance points
    results = calculatePerformancePoints(results, params);
    
    % Generate plots and visualizations
    generatePlots(results, params);
    
    % Display results
    displayResults(results);
    
    fprintf('\n=====================================\n');
    fprintf('Simulation Complete!\n');
    fprintf('=====================================\n');
end

%% INITIALIZATION FUNCTIONS
function initializeGlobalVariables()
    % Initialize global variables for tire models
    global tire_coeff_lat tire_coeff_lon mulat mulon
    
    % Hoosier R25B 16x7.5x10 IA = 0deg P = 12psi - Lateral coefficients
    tire_coeff_lat = [1.497564443, -0.002272231, 2.840472825, 70.86957406, ...
                      0.222001058, 369690, -3.24E-07, 0.000447167, ...
                      -0.003834695, 0.002574449, -0.079271384, -6.855006239];
    
    % Hoosier R25B 16x7.5x10 IA = 0deg P = 12psi - Longitudinal coefficients
    tire_coeff_lon = [1.2309, -0.0027, 2.9719, 1.1974, 0.0596, 4.6389e+04, ...
                      -0.0140, 11.4756, 253.5530, 138.2941, 0.0003, -4.5750];
    
    % Grip modification factors
    mulat = 0.65;  % Lateral grip factor
    mulon = 0.65;  % Longitudinal grip factor
end

function params = setupSimulationParameters()
    % Set up all simulation parameters
    params = struct();
    
    % Simulation settings
    params.batch = 0;  % 0=single run, 1=1D sweep, 2=2D sweep
    params.dt = 0.001;  % Time step (seconds)
    params.roundoff = 3;  % Decimal places for rounding
    params.v_tol = 0.001;  % Velocity convergence tolerance (m/s)
    params.a_tol = 0.0001;  % Acceleration convergence tolerance (g)
    
    % Tire model parameters
    params.SA_max = 15;  % Maximum slip angle (degrees)
    params.SR_max = 0.25;  % Maximum slip ratio
    params.SA_res = params.SA_max/50;  % Slip angle resolution
    params.SR_res = params.SR_max/50;  % Slip ratio resolution
    params.SA_range = 0:params.SA_res:params.SA_max;
    
    % Velocity range for vehicle model
    params.v_min = 5.25;  % Minimum velocity (m/s)
    params.v_max = 35.0;  % Maximum velocity (m/s)
    params.dv = 0.05;  % Velocity increment (m/s)
    params.v_range = params.v_min:params.dv:params.v_max;
    
    % DRS (Drag Reduction System) parameters
    params.drs_enabled = true;  % Enable/disable DRS
    params.drs_drag_reduction = 0.30;  % 30% drag reduction when active
    params.drs_downforce_loss = 0.10;  % 10% downforce loss when active
    params.g_thresh_g = 0.90;  % Lateral G threshold for DRS deactivation
    params.g_thresh = params.g_thresh_g * 9.81;  % Convert to m/s^2
    
    return;
end

function [track_data, params] = setupTrackData(params)
    % Create a sample track or load track data
    % This creates a simple test track with corners and straights
    
    fprintf('Setting up track data...\n');
    
    % Example track: alternating straights and corners
    % Format: r = radius (0 for straight), d = distance/arc length
    track_segments = [
        0, 100;   % 100m straight
        30, 50;   % 50m arc with 30m radius
        0, 80;    % 80m straight
        20, 40;   % 40m arc with 20m radius
        0, 120;   % 120m straight
        25, 60;   % 60m arc with 25m radius
        0, 90;    % 90m straight
        35, 70;   % 70m arc with 35m radius
    ];
    
    track_data.r = track_segments(:, 1)';  % Radius array
    track_data.d = track_segments(:, 2)';  % Distance array
    track_data.distance_total = sum(track_data.d);
    track_data.num_segments = length(track_data.d);
    
    % Calculate endurance laps (based on 22km endurance distance)
    params.enduro_laps = round(22000 / track_data.distance_total);
    
    fprintf('Track loaded: %d segments, %.1f m total distance\n', ...
            track_data.num_segments, track_data.distance_total);
    
    return;
end

function vehicle = setupVehicleParameters(params)
    % Set up vehicle parameters
    vehicle = struct();
    
    % Mass parameters (convert from lbs to kg)
    vehicle.m_driver = 180 * 0.453592;  % Driver mass (kg)
    vehicle.m_car = 280 * 0.453592;     % Car mass (kg)
    vehicle.m_accum = 60 * 0.453592;    % Accumulator mass (kg)
    vehicle.m_DRS = 1 * 0.453592;       % DRS system mass (kg)
    vehicle.m_total = vehicle.m_car + vehicle.m_driver + ...
                      vehicle.m_accum + vehicle.m_DRS;
    
    % Chassis geometry (convert from inches to meters)
    vehicle.track = 47 * 0.0254;     % Track width (m)
    vehicle.l = 60.25 * 0.0254;      % Wheelbase (m)
    vehicle.cg_h = 11.2 * 0.0254;    % CG height (m)
    vehicle.wdf = 0.445;              % Weight distribution front (0-1)
    vehicle.adf = 0.46;               % Aero downforce distribution front (0-1)
    
    % Calculate front and rear axle distances from CG
    vehicle.a = vehicle.l * (1 - vehicle.wdf);  % Front axle to CG
    vehicle.b = vehicle.l * vehicle.wdf;        % Rear axle to CG
    
    % Drivetrain
    vehicle.finaldrive = 4;           % Final drive ratio
    vehicle.r_tire = 15.657/2 * 0.0254;  % Tire loaded radius (m)
    
    % Aerodynamics (convert from lbs to N)
    vehicle.Fl_base = 111 * 4.44822;  % Baseline downforce at 55 kph (N)
    vehicle.Fd_base = 45 * 4.44822;   % Baseline drag at 55 kph (N)
    vehicle.area = 1.15;               % Frontal area (m^2)
    vehicle.rho = 1.204;               % Air density (kg/m^3)
    
    % Calculate aerodynamic coefficients (referenced at 55 kph = 15.6464 m/s)
    vehicle.c_lift_base = 2 * vehicle.Fl_base / ...
                          (vehicle.rho * vehicle.area * 15.6464^2);
    vehicle.c_drag_base = 2 * vehicle.Fd_base / ...
                          (vehicle.rho * vehicle.area * 15.6464^2);
    
    % Engine/Motor parameters
    vehicle.power_coeff = 1.0;        % Power scaling coefficient
    vehicle.shift_time = 0.1;         % Shift time (s)
    vehicle.rpm_limit = 5500;          % RPM limit
    
    return;
end

function engine = setupEngineData(vehicle, params)
    % Set up engine/motor data
    % Creates a synthetic motor torque curve if no data file exists
    
    fprintf('Setting up engine/motor data...\n');
    
    engine = struct();
    engine.name = 'Emrax 208 HV Electric Motor';
    
    % Create synthetic motor data (typical electric motor curve)
    engine.rpm = 0:100:6000;  % RPM range
    
    % Electric motor torque curve (relatively flat with dropoff at high RPM)
    peak_torque = 120;  % Nm
    engine.torque = peak_torque * ones(size(engine.rpm));
    high_rpm_idx = engine.rpm > 3500;
    engine.torque(high_rpm_idx) = peak_torque * ...
        (1 - 0.5 * (engine.rpm(high_rpm_idx) - 3500) / 2500);
    
    % Calculate power from torque
    engine.power = engine.torque .* engine.rpm * 2 * pi / 60 / 1000;  % kW
    
    % Gearing (single reduction for electric motor)
    engine.gearing = [1, 1];  % Primary, final handled separately
    engine.shifting = [];      % No shifting for single-speed
    
    % Calculate force at wheels for each velocity
    engine.Fx_engine = zeros(size(params.v_range));
    for i = 1:length(params.v_range)
        v = params.v_range(i);
        rpm = v * vehicle.finaldrive * 60 / (2 * pi) / vehicle.r_tire;
        
        if rpm > vehicle.rpm_limit
            engine.Fx_engine(i) = 0;
        else
            % Interpolate torque at this RPM
            torque_at_rpm = interp1(engine.rpm, engine.torque, ...
                                   min(rpm, max(engine.rpm)), 'linear', 0);
            engine.Fx_engine(i) = torque_at_rpm * vehicle.power_coeff * ...
                                 vehicle.finaldrive / vehicle.r_tire;
        end
    end
    
    fprintf('Engine setup complete: %s\n', engine.name);
    
    return;
end

%% MAIN SIMULATION FUNCTION
function results = runMainSimulation(track_data, vehicle, engine, params)
    fprintf('\n=====================================\n');
    fprintf('Running main simulation...\n');
    fprintf('=====================================\n\n');
    
    % Initialize results structure
    results = struct();
    
    % Generate vehicle models (with and without DRS if enabled)
    fprintf('Generating vehicle dynamics models...\n');
    [models_on, models_off] = generateVehicleModels(vehicle, engine, params);
    
    % Simulate individual events
    fprintf('\nSimulating events:\n');
    fprintf('------------------\n');
    
    % 1. Acceleration simulation
    fprintf('1. Acceleration (0-75m)... ');
    results.accel = simulateAcceleration(models_on, params);
    fprintf('Time: %.3f s\n', results.accel.time);
    
    % 2. Skidpad simulation
    fprintf('2. Skidpad... ');
    results.skidpad = simulateSkidpad(models_on, vehicle, params);
    fprintf('Time: %.3f s\n', results.skidpad.time);
    
    % 3. Lap simulation (autocross/endurance)
    fprintf('3. Track lap simulation... ');
    results.lap = simulateLap(track_data, models_on, models_off, ...
                             vehicle, params);
    fprintf('Lap time: %.3f s\n', results.lap.laptime);
    
    % 4. Endurance calculation
    results.endurance = struct();
    results.endurance.laps = params.enduro_laps;
    results.endurance.time = results.lap.laptime * params.enduro_laps;
    fprintf('4. Endurance (%d laps)... Time: %.3f s\n', ...
            params.enduro_laps, results.endurance.time);
    
    return;
end

function [models_on, models_off] = generateVehicleModels(vehicle, engine, params)
    % Generate vehicle dynamics models with DRS on and off
    
    % DRS ON model
    if params.drs_enabled
        Fd = vehicle.Fd_base * (1 - params.drs_drag_reduction);
        Fl = vehicle.Fl_base * (1 - params.drs_downforce_loss);
    else
        Fd = vehicle.Fd_base;
        Fl = vehicle.Fl_base;
    end
    
    c_lift = 2 * Fl / (vehicle.rho * vehicle.area * 15.6464^2);
    c_drag = 2 * Fd / (vehicle.rho * vehicle.area * 15.6464^2);
    
    lift_on = @(v) 0.5 * c_lift * vehicle.rho * vehicle.area * v.^2;
    drag_on = @(v) 0.5 * c_drag * vehicle.rho * vehicle.area * v.^2;
    
    models_on = generateSingleModel(vehicle, engine, params, lift_on, drag_on);
    
    % DRS OFF model
    Fd = vehicle.Fd_base;
    Fl = vehicle.Fl_base;
    
    c_lift = 2 * Fl / (vehicle.rho * vehicle.area * 15.6464^2);
    c_drag = 2 * Fd / (vehicle.rho * vehicle.area * 15.6464^2);
    
    lift_off = @(v) 0.5 * c_lift * vehicle.rho * vehicle.area * v.^2;
    drag_off = @(v) 0.5 * c_drag * vehicle.rho * vehicle.area * v.^2;
    
    models_off = generateSingleModel(vehicle, engine, params, lift_off, drag_off);
    
    return;
end

function model = generateSingleModel(vehicle, engine, params, lift, drag)
    % Generate a single vehicle dynamics model
    
    g = 9.81;
    
    % Preallocate arrays
    Ay_out = zeros(size(params.v_range));
    Ax_drive_out = zeros(size(params.v_range));
    Ax_brake_out = zeros(size(params.v_range));
    alpha_out = zeros(length(params.v_range), 4);
    
    for i = 1:length(params.v_range)
        v = params.v_range(i);
        fdowns = lift(v);
        fdrag = drag(v);
        
        % LATERAL ACCELERATION CALCULATION
        Ay_in = 0;
        Ay_last = 10;
        
        while abs(Ay_in - Ay_last) >= params.a_tol
            Ay_last = Ay_in;
            
            % Lateral load transfer
            LT_f = Ay_last * vehicle.m_total * g * vehicle.cg_h * ...
                   vehicle.wdf / vehicle.track;
            LT_r = Ay_last * vehicle.m_total * g * vehicle.cg_h * ...
                   (1 - vehicle.wdf) / vehicle.track;
            
            % Corner radii for each wheel
            if Ay_last > 0
                R_base = v^2 / (Ay_last * g);
                R1 = R_base - vehicle.track/2;  % Left front
                R2 = R_base + vehicle.track/2;  % Right front
                R3 = R_base - vehicle.track/2;  % Left rear
                R4 = R_base + vehicle.track/2;  % Right rear
                
                % Heading angles
                heading1 = atand(vehicle.l * (1 - vehicle.wdf) / R1);
                heading2 = atand(vehicle.l * (1 - vehicle.wdf) / R2);
                heading3 = atand(vehicle.l * vehicle.wdf / R3);
                heading4 = atand(vehicle.l * vehicle.wdf / R4);
                
                SA_delta_f = heading1 - heading2;
                SA_delta_r = heading3 - heading4;
            else
                SA_delta_f = 0;
                SA_delta_r = 0;
            end
            
            % Static corner weights (N)
            w_1 = 0.5 * vehicle.m_total * g * vehicle.wdf;
            w_2 = 0.5 * vehicle.m_total * g * vehicle.wdf;
            w_3 = 0.5 * vehicle.m_total * g * (1 - vehicle.wdf);
            w_4 = 0.5 * vehicle.m_total * g * (1 - vehicle.wdf);
            
            % Normal forces on each tire (convert to lbf for tire model)
            Fz_1 = (w_1 + 0.5 * fdowns * vehicle.adf - LT_f) / 4.44822;
            Fz_2 = (w_2 + 0.5 * fdowns * vehicle.adf + LT_f) / 4.44822;
            Fz_3 = (w_3 + 0.5 * fdowns * (1 - vehicle.adf) - LT_r) / 4.44822;
            Fz_4 = (w_4 + 0.5 * fdowns * (1 - vehicle.adf) + LT_r) / 4.44822;
            
            % Ensure positive normal forces
            Fz_1 = max(0, Fz_1);
            Fz_2 = max(0, Fz_2);
            Fz_3 = max(0, Fz_3);
            Fz_4 = max(0, Fz_4);
            
            % Calculate lateral forces using tire model
            [Fy_1, alpha_1] = calculateFyMax(Fz_1, params.SA_max, params.SA_res);
            [Fy_2, alpha_2] = calculateFyMax(Fz_2, params.SA_max, params.SA_res);
            
            % Rear tires with slip angle difference
            Fy_range_3 = calculateFyRange(Fz_3, params.SA_range + SA_delta_r);
            Fy_range_4 = calculateFyRange(Fz_4, params.SA_range);
            [Fy_rear, SA_r] = max(Fy_range_3 + Fy_range_4);
            Fy_3 = Fy_range_3(SA_r);
            Fy_4 = Fy_range_4(SA_r);
            alpha_3 = params.SA_range(SA_r) + SA_delta_r;
            alpha_4 = params.SA_range(SA_r);
            
            % Moment balance
            Mz_cg_F = (Fy_1 + Fy_2) * (1 - vehicle.wdf);
            Mz_cg_R = (Fy_3 + Fy_4) * vehicle.wdf;
            
            % Scale forces to maintain moment balance
            if Mz_cg_F > Mz_cg_R
                Fy_F_scale = Mz_cg_R / ((1 - vehicle.wdf) * (Fy_1 + Fy_2));
                Fy_R_scale = 1;
            elseif Mz_cg_F < Mz_cg_R
                Fy_F_scale = 1;
                Fy_R_scale = Mz_cg_F / (vehicle.wdf * (Fy_3 + Fy_4));
            else
                Fy_F_scale = 1;
                Fy_R_scale = 1;
            end
            
            % Total lateral acceleration (convert from lbf to N)
            Ay_in = (Fy_1 * Fy_F_scale * 4.44822 + Fy_2 * Fy_F_scale * 4.44822 + ...
                    Fy_3 * Fy_R_scale * 4.44822 + Fy_4 * Fy_R_scale * 4.44822) / ...
                    (vehicle.m_total * g);
        end
        
        alpha_out(i, :) = [alpha_1, alpha_2, alpha_3, alpha_4];
        Ay_out(i) = Ay_in * g;
        
        % DRIVING ACCELERATION CALCULATION
        Ax_drive_in = 0;
        Ax_last = 10;
        
        while abs(Ax_drive_in - Ax_last) > params.a_tol
            Ax_last = Ax_drive_in;
            
            % Longitudinal load transfer
            LT = g * vehicle.m_total * Ax_last * vehicle.cg_h / (vehicle.l * 2);
            
            % Normal forces (convert to lbf)
            Fz_1 = (w_1 + 0.5 * fdowns * vehicle.adf - LT) / 4.44822;
            Fz_2 = (w_2 + 0.5 * fdowns * vehicle.adf - LT) / 4.44822;
            Fz_3 = (w_3 + 0.5 * fdowns * (1 - vehicle.adf) + LT) / 4.44822;
            Fz_4 = (w_4 + 0.5 * fdowns * (1 - vehicle.adf) + LT) / 4.44822;
            
            % Ensure positive normal forces
            Fz_1 = max(0, Fz_1);
            Fz_2 = max(0, Fz_2);
            Fz_3 = max(0, Fz_3);
            Fz_4 = max(0, Fz_4);
            
            % Rear wheel drive - calculate traction limit
            Fx_3 = calculateFxDrive(Fz_3, params.SR_max, params.SR_res);
            Fx_4 = calculateFxDrive(Fz_4, params.SR_max, params.SR_res);
            
            Fx_tire = Fx_3 * 4.44822 + Fx_4 * 4.44822;
            
            % Limited by either traction or engine power
            if Fx_tire >= engine.Fx_engine(i)
                Ax_drive_in = (engine.Fx_engine(i) - fdrag) / (vehicle.m_total * g);
            else
                Ax_drive_in = (Fx_tire - fdrag) / (vehicle.m_total * g);
            end
        end
        
        Ax_drive_out(i) = Ax_drive_in * g;
        
        % BRAKING ACCELERATION CALCULATION
        Ax_brake_in = 0;
        Ax_last = 10;
        
        while abs(Ax_brake_in - Ax_last) > params.a_tol
            Ax_last = Ax_brake_in;
            
            % Longitudinal load transfer
            LT = g * vehicle.m_total * Ax_last * vehicle.cg_h / (vehicle.l * 2);
            
            % Normal forces (convert to lbf)
            Fz_1 = (w_1 + 0.5 * fdowns * vehicle.adf - LT) / 4.44822;
            Fz_2 = (w_2 + 0.5 * fdowns * vehicle.adf - LT) / 4.44822;
            Fz_3 = (w_3 + 0.5 * fdowns * (1 - vehicle.adf) + LT) / 4.44822;
            Fz_4 = (w_4 + 0.5 * fdowns * (1 - vehicle.adf) + LT) / 4.44822;
            
            % Ensure positive normal forces
            Fz_1 = max(0, Fz_1);
            Fz_2 = max(0, Fz_2);
            Fz_3 = max(0, Fz_3);
            Fz_4 = max(0, Fz_4);
            
            % All wheel braking
            Fx_1 = -calculateFxBrake(Fz_1, params.SR_max, params.SR_res);
            Fx_2 = -calculateFxBrake(Fz_2, params.SR_max, params.SR_res);
            Fx_3 = -calculateFxBrake(Fz_3, params.SR_max, params.SR_res);
            Fx_4 = -calculateFxBrake(Fz_4, params.SR_max, params.SR_res);
            
            Fx_tire = Fx_1 * 4.44822 + Fx_2 * 4.44822 + ...
                     Fx_3 * 4.44822 + Fx_4 * 4.44822;
            
            Ax_brake_in = (Fx_tire - fdrag) / (vehicle.m_total * g);
        end
        
        Ax_brake_out(i) = Ax_brake_in * g;
    end
    
    % Fit polynomials to create smooth functions
    P1 = polyfit(params.v_range, Ay_out, 3);
    P2 = polyfit(params.v_range, Ax_drive_out, 10);
    P3 = polyfit(params.v_range, Ax_brake_out, 3);
    
    model.Ay = @(x) polyval(P1, x);
    model.Ax_drive = @(x) polyval(P2, x);
    model.Ax_brake = @(x) polyval(P3, x);
    model.alpha_out = alpha_out;
    model.lift = lift;
    model.drag = drag;
    
    return;
end

%% EVENT SIMULATION FUNCTIONS
function accel = simulateAcceleration(models, params)
    % Simulate 0-75m acceleration run
    
    accel = struct();
    accel.time = 0;
    accel.distance = zeros(1, 1);
    accel.velocity = zeros(1, 1);
    accel.acceleration = zeros(1, 1);
    
    % Initial conditions
    w = 1;
    accel.distance(1) = 0;
    accel.velocity(1) = 5;  % Start at 5 m/s to avoid low-speed issues
    accel.acceleration(1) = 0;
    
    % Simulate until 75m
    while accel.distance(w) <= 75
        accel.time = accel.time + params.dt;
        
        % Get acceleration at current velocity
        accel.acceleration(w+1) = abs(models.Ax_drive(accel.velocity(w)));
        
        % Update velocity and position using trapezoidal integration
        accel.velocity(w+1) = accel.velocity(w) + ...
            0.5 * params.dt * (accel.acceleration(w+1) + accel.acceleration(w));
        accel.distance(w+1) = accel.distance(w) + ...
            0.5 * params.dt * (accel.velocity(w+1) + accel.velocity(w));
        
        w = w + 1;
        
        % Safety check
        if accel.time > 20
            warning('Acceleration simulation timeout - check vehicle parameters');
            break;
        end
    end
    
    return;
end

function skidpad = simulateSkidpad(models, vehicle, params)
    % Simulate skidpad event (constant radius cornering)
    
    skidpad = struct();
    
    % Skidpad geometry (FSAE standard)
    r_skid = 7.625 + vehicle.track/2;  % Path radius (m)
    d_skid = r_skid * 2 * pi;          % Full circle distance
    
    % Find steady-state cornering speed
    v_check = 0;
    corner_conv = 0;
    
    while corner_conv ~= 1
        % Simulate corner at test velocity
        [v_corner, time_corner] = simulateCorner(models.Ay(v_check), ...
                                                 r_skid, d_skid, params);
        v_i = mean(v_corner);
        
        % Check convergence
        if abs(v_check - v_i) >= params.v_tol
            v_check = abs(v_check + v_i) / 2;
        else
            corner_conv = 1;
        end
    end
    
    skidpad.time = max(time_corner);
    skidpad.velocity = v_corner;
    skidpad.avg_velocity = mean(v_corner);
    skidpad.lateral_g = skidpad.avg_velocity^2 / (r_skid * 9.81);
    
    return;
end

function lap = simulateLap(track_data, models_on, models_off, vehicle, params)
    % Simulate a complete lap around the track
    
    lap = struct();
    num_segments = track_data.num_segments;
    
    % Preallocate output arrays
    max_points = 100000;  % Maximum points per segment
    time_out = zeros(num_segments, max_points);
    v_out = zeros(num_segments, max_points);
    throttle_out = zeros(num_segments, max_points);
    segment_times = zeros(1, num_segments);
    drs_active = ones(1, num_segments);  % Assume DRS on by default
    
    % Simulate each segment
    for i = 1:num_segments
        if track_data.r(i) > 0  % Corner segment
            % Determine if DRS should be active based on lateral G
            v_check = 0;
            corner_conv = 0;
            
            % First try with DRS on
            models = models_on;
            
            while corner_conv ~= 1
                [v_c, time_c, throttle_c] = simulateCorner(models.Ay(v_check), ...
                    track_data.r(i), track_data.d(i), params);
                v_i = mean(v_c);
                
                if abs(v_check - v_i) >= params.v_tol
                    v_check = abs(v_check + v_i) / 2;
                else
                    corner_conv = 1;
                end
            end
            
            % Check lateral G
            lateral_g = v_i^2 / track_data.r(i);
            
            % If lateral G exceeds threshold, re-simulate with DRS off
            if lateral_g > params.g_thresh && params.drs_enabled
                drs_active(i) = 0;
                models = models_off;
                v_check = 0;
                corner_conv = 0;
                
                while corner_conv ~= 1
                    [v_c, time_c, throttle_c] = simulateCorner(models.Ay(v_check), ...
                        track_data.r(i), track_data.d(i), params);
                    v_i = mean(v_c);
                    
                    if abs(v_check - v_i) >= params.v_tol
                        v_check = abs(v_check + v_i) / 2;
                    else
                        corner_conv = 1;
                    end
                end
            end
            
            % Store results
            n_points = length(time_c);
            time_out(i, 1:n_points) = time_c;
            v_out(i, 1:n_points) = min(v_c, params.v_max);
            throttle_out(i, 1:n_points) = throttle_c;
            segment_times(i) = max(time_c);
            
        else  % Straight segment
            % Determine entry and exit velocities from neighboring corners
            [v_entry, v_exit] = determineStraitVelocities(i, track_data, ...
                                                          models_on, params);
            
            % Use DRS on straights if enabled
            if params.drs_enabled
                models = models_on;
            else
                models = models_off;
            end
            
            % Simulate straight
            [time_s, v_s, throttle_s] = simulateStraight(track_data.d(i), ...
                params.dt, models.Ax_drive, models.Ax_brake, v_exit, v_entry, params);
            
            % Store results
            n_points = length(time_s);
            time_out(i, 1:n_points) = time_s;
            v_out(i, 1:n_points) = v_s;
            throttle_out(i, 1:n_points) = throttle_s;
            segment_times(i) = max(time_s);
        end
    end
    
    % Compile lap results
    lap.laptime = sum(segment_times);
    lap.time_out = time_out;
    lap.v_out = v_out;
    lap.throttle_out = throttle_out;
    lap.segment_times = segment_times;
    lap.drs_active = drs_active;
    
    % Create time and velocity vectors for plotting
    lap.time_vec = [];
    lap.speed_vec = [];
    lap.throttle_vec = [];
    
    cumulative_time = 0;
    for i = 1:num_segments
        segment_data = time_out(i, :);
        nonzero_idx = segment_data > 0;
        segment_time = segment_data(nonzero_idx) + cumulative_time;
        
        lap.time_vec = [lap.time_vec, segment_time];
        lap.speed_vec = [lap.speed_vec, v_out(i, nonzero_idx)];
        lap.throttle_vec = [lap.throttle_vec, throttle_out(i, nonzero_idx)];
        
        cumulative_time = cumulative_time + segment_times(i);
    end
    
    return;
end

function [v_entry, v_exit] = determineStraitVelocities(idx, track_data, models, params)
    % Determine entry and exit velocities for a straight segment
    
    num_segments = length(track_data.d);
    
    % Find previous and next corner indices
    if idx == 1
        prev_idx = num_segments;
        next_idx = idx + 1;
    elseif idx == num_segments
        prev_idx = idx - 1;
        next_idx = 1;
    else
        prev_idx = idx - 1;
        next_idx = idx + 1;
    end
    
    % Calculate exit velocity from previous segment
    if track_data.r(prev_idx) > 0
        v_check = 0;
        corner_conv = 0;
        while corner_conv ~= 1
            v_last = simulateCornerSimple(models.Ay(v_check), ...
                track_data.r(prev_idx), track_data.d(prev_idx), params);
            v_i = mean(v_last);
            if abs(v_check - v_i) >= params.v_tol
                v_check = abs(v_check + v_i) / 2;
            else
                corner_conv = 1;
            end
        end
        v_exit = mean(v_last);
    else
        v_exit = params.v_min;  % Default if previous is also straight
    end
    
    % Calculate entry velocity to next segment
    if track_data.r(next_idx) > 0
        v_check = 0;
        corner_conv = 0;
        while corner_conv ~= 1
            v_next = simulateCornerSimple(models.Ay(v_check), ...
                track_data.r(next_idx), track_data.d(next_idx), params);
            v_i = mean(v_next);
            if abs(v_check - v_i) >= params.v_tol
                v_check = abs(v_check + v_i) / 2;
            else
                corner_conv = 1;
            end
        end
        v_entry = mean(v_next);
    else
        v_entry = params.v_max;  % Default if next is also straight
    end
    
    return;
end

%% CORNER AND STRAIGHT SIMULATION FUNCTIONS
function [v_, t_, throttle] = simulateCorner(Ay, r, d, params)
    % Simulate cornering at constant radius
    
    v = sqrt(Ay * r);  % Steady-state cornering speed
    t = d / v;         % Time to complete corner
    t = round(t * 10^params.roundoff) / 10^params.roundoff;
    
    t_ = 0:params.dt:t;
    v_ = ones(1, length(t_)) * v;
    throttle = ones(1, length(t_)) * 0.3;  % Partial throttle in corners
    
    return;
end

function v_ = simulateCornerSimple(Ay, r, d, params)
    % Simple corner simulation for velocity estimation
    
    v = sqrt(Ay * r);
    t = d / v;
    t = round(t * 10^params.roundoff) / 10^params.roundoff;
    
    t_ = 0:params.dt:t;
    v_ = ones(1, length(t_)) * v;
    
    return;
end

function [time, velocity, throttle] = simulateStraight(dist, dt, Ax_drive, ...
                                                        Ax_brake, v_initial, ...
                                                        v_final, params)
    % Simulate straight with optimal braking point
    
    % Forward integration (acceleration)
    t_d(1) = 0;
    x_drive(1) = 0;
    xd_drive(1) = v_initial;
    xdd_drive(1) = 0;
    j = 1;
    
    while x_drive(j) <= dist
        t_d(j+1) = t_d(j) + dt;
        xdd_drive(j+1) = Ax_drive(xd_drive(j));
        xd_drive(j+1) = xd_drive(j) + 0.5 * (xdd_drive(j+1) + xdd_drive(j)) * dt;
        x_drive(j+1) = x_drive(j) + 0.5 * (xd_drive(j+1) + xd_drive(j)) * dt;
        j = j + 1;
        
        % Safety check
        if j > 1000000
            break;
        end
    end
    
    % Backward integration (braking)
    k = 1;
    t_b(k) = t_d(j-1);
    x_brake(k) = dist;
    xd_brake(k) = v_final;
    xdd_brake(k) = 0;
    
    while x_brake(k) >= 0
        t_b(k+1) = t_b(k) - dt;
        xdd_brake(k+1) = -Ax_brake(xd_brake(k));
        xd_brake(k+1) = xd_brake(k) + 0.5 * (xdd_brake(k+1) + xdd_brake(k)) * dt;
        x_brake(k+1) = x_brake(k) - 0.5 * (xd_brake(k+1) + xd_brake(k)) * dt;
        k = k + 1;
        
        % Safety check
        if k > 1000000
            break;
        end
    end
    
    % Trim arrays to same length
    ii_max = min(length(xd_drive), length(xd_brake));
    xd_drive = xd_drive(1:ii_max);
    xd_brake = xd_brake(1:ii_max);
    xd_brake_flip = fliplr(xd_brake);
    
    % Determine optimal velocity profile
    throttle = zeros(1, ii_max);
    velocity = zeros(1, ii_max);
    time = zeros(1, ii_max);
    
    for ii = 1:ii_max-1
        if xd_drive(ii) >= xd_brake_flip(ii)
            % Braking
            throttle(ii) = 0;
            velocity(ii) = xd_brake_flip(ii);
        else
            % Accelerating
            throttle(ii) = 1;
            velocity(ii) = xd_drive(ii);
        end
        time(ii+1) = time(ii) + dt;
    end
    
    return;
end

%% TIRE MODEL FUNCTIONS
function Y = pacejkaModel(beta, in)
    % Pacejka '93 tire model
    
    Fz = in(1);  % Normal force
    X = in(2);   % Slip angle or slip ratio
    mu = in(3);  % Friction coefficient
    
    % Extract coefficients
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
    
    % Calculate tire force
    D = mu * (c1 * Fz^2 + c2 * Fz);
    BCD = c3 * sind(c4 * atand(c5 * Fz));
    B = BCD / (C * D);
    x = X + SH;
    E = (c6 * Fz^2 + c7 * Fz + c8) + dE * sign(x);
    y = D * sind(C * atand(B * x - E * (B * x - atand(B * x))));
    Y0 = y + SV;
    Y0(isnan(Y0)) = 0;
    Y = Y0;
    
    return;
end

function [max_Fy, alpha] = calculateFyMax(Fz, SA_max, res)
    % Calculate maximum lateral force and optimal slip angle
    global tire_coeff_lat mulat
    
    SA = 0:res:SA_max;
    Fy = zeros(size(SA));
    
    for i = 1:length(SA)
        Fy(i) = pacejkaModel(tire_coeff_lat, [Fz, SA(i), mulat]);
    end
    
    [max_Fy, I] = max(Fy);
    alpha = SA(I);
    
    return;
end

function Fy = calculateFyRange(Fz, SA_range)
    % Calculate lateral forces over a range of slip angles
    global tire_coeff_lat mulat
    
    Fy = zeros(size(SA_range));
    
    for i = 1:length(SA_range)
        Fy(i) = pacejkaModel(tire_coeff_lat, [Fz, SA_range(i), mulat]);
    end
    
    return;
end

function max_Fx = calculateFxDrive(Fz, SR_max, res)
    % Calculate maximum driving force
    global tire_coeff_lon mulon
    
    SR = 0:res:SR_max;
    Fx = zeros(size(SR));
    
    for i = 1:length(SR)
        Fx(i) = pacejkaModel(tire_coeff_lon, [Fz, SR(i), mulon]);
    end
    
    max_Fx = max(Fx);
    
    return;
end

function max_Fx = calculateFxBrake(Fz, SR_max, res)
    % Calculate maximum braking force
    global tire_coeff_lon mulon
    
    SR = 0:-res:-SR_max;
    Fx = zeros(size(SR));
    
    for i = 1:length(SR)
        Fx(i) = pacejkaModel(tire_coeff_lon, [Fz, SR(i), mulon]);
    end
    
    max_Fx = -min(Fx);
    
    return;
end

%% PERFORMANCE POINTS CALCULATION
function results = calculatePerformancePoints(results, params)
    % Calculate FSAE/Formula Student competition points
    
    % Reference times (these would normally come from competition data)
    % Using typical values for demonstration
    t_accel_min = 3.5;     % Best acceleration time
    t_skidpad_min = 4.8;   % Best skidpad time
    t_autocross_min = 45;  % Best autocross time
    t_enduro_min = results.endurance.time * 0.85;  % Best endurance time
    
    % Acceleration points (75.5 points max)
    if results.accel.time <= t_accel_min * 1.5
        results.points.accel = 71.5 * ((t_accel_min * 1.5 / results.accel.time) - 1) / 0.5 + 3.5;
    else
        results.points.accel = 3.5;
    end
    
    % Skidpad points (75 points max)
    if results.skidpad.time <= t_skidpad_min * 1.25
        results.points.skidpad = 71.5 * (((t_skidpad_min * 1.25 / results.skidpad.time)^2 - 1) / 0.5625) + 3.5;
    else
        results.points.skidpad = 3.5;
    end
    
    % Autocross points (100 points max)
    if results.lap.laptime <= t_autocross_min * 1.25
        results.points.autocross = 95.5 * (((t_autocross_min * 1.25 / results.lap.laptime) - 1) / 0.25) + 4.5;
    else
        results.points.autocross = 4.5;
    end
    
    % Endurance points (325 points max)
    if results.endurance.time <= t_enduro_min * 1.333
        results.points.endurance = 300 * (((t_enduro_min * 1.333 / results.endurance.time) - 1) / 0.333) + 25;
    else
        results.points.endurance = 25;
    end
    
    % Total dynamic event points
    results.points.total = results.points.accel + results.points.skidpad + ...
                           results.points.autocross + results.points.endurance;
    
    % Ensure points are non-negative and within limits
    results.points.accel = max(0, min(75.5, results.points.accel));
    results.points.skidpad = max(0, min(75, results.points.skidpad));
    results.points.autocross = max(0, min(100, results.points.autocross));
    results.points.endurance = max(0, min(325, results.points.endurance));
    
    return;
end

%% PLOTTING AND VISUALIZATION
function generatePlots(results, params)
    % Generate visualization plots
    
    % Figure 1: Speed vs Time
    figure('Name', 'Lap Simulation Results', 'NumberTitle', 'off');
    subplot(3, 1, 1);
    plot(results.lap.time_vec, results.lap.speed_vec, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    title('Speed vs Time');
    ylim([0, params.v_max * 1.1]);
    
    % Throttle position
    subplot(3, 1, 2);
    plot(results.lap.time_vec, results.lap.throttle_vec, 'r-', 'LineWidth', 1);
    grid on;
    xlabel('Time (s)');
    ylabel('Throttle Position');
    title('Throttle vs Time');
    ylim([-0.1, 1.1]);
    
    % DRS activation
    if params.drs_enabled
        subplot(3, 1, 3);
        segment_centers = cumsum(results.lap.segment_times) - ...
                         results.lap.segment_times/2;
        bar(segment_centers, results.lap.drs_active, 'FaceColor', [0.2, 0.7, 0.2]);
        grid on;
        xlabel('Time (s)');
        ylabel('DRS Active');
        title('DRS Activation by Segment');
        ylim([-0.1, 1.1]);
    end
    
    % Figure 2: Performance Summary
    figure('Name', 'Performance Summary', 'NumberTitle', 'off');
    
    % Event times bar chart
    subplot(2, 2, 1);
    event_times = [results.accel.time, results.skidpad.time, ...
                   results.lap.laptime, results.endurance.time/60];
    event_names = {'Accel', 'Skidpad', 'Autocross', 'Endurance (min)'};
    bar(event_times, 'FaceColor', [0.3, 0.5, 0.8]);
    set(gca, 'XTickLabel', event_names);
    ylabel('Time');
    title('Event Times');
    grid on;
    
    % Points breakdown
    subplot(2, 2, 2);
    points = [results.points.accel, results.points.skidpad, ...
              results.points.autocross, results.points.endurance];
    bar(points, 'FaceColor', [0.8, 0.3, 0.3]);
    set(gca, 'XTickLabel', event_names);
    ylabel('Points');
    title('Competition Points');
    grid on;
    
    % Speed histogram
    subplot(2, 2, 3);
    histogram(results.lap.speed_vec, 30, 'FaceColor', [0.3, 0.7, 0.3]);
    xlabel('Speed (m/s)');
    ylabel('Frequency');
    title('Speed Distribution');
    grid on;
    
    % Lateral G utilization (from skidpad)
    subplot(2, 2, 4);
    text(0.5, 0.5, sprintf('Peak Lateral G: %.2f g\n\nTotal Points: %.1f', ...
         results.skidpad.lateral_g, results.points.total), ...
         'HorizontalAlignment', 'center', 'FontSize', 14);
    axis off;
    title('Performance Metrics');
    
    return;
end

%% RESULTS DISPLAY
function displayResults(results)
    % Display simulation results in command window
    
    fprintf('\n=====================================\n');
    fprintf('SIMULATION RESULTS\n');
    fprintf('=====================================\n\n');
    
    fprintf('Event Times:\n');
    fprintf('------------\n');
    fprintf('Acceleration (0-75m):  %6.3f s\n', results.accel.time);
    fprintf('Skidpad:               %6.3f s\n', results.skidpad.time);
    fprintf('Autocross (1 lap):     %6.3f s\n', results.lap.laptime);
    fprintf('Endurance (%d laps):  %6.3f s\n', results.endurance.laps, ...
            results.endurance.time);
    
    fprintf('\nPerformance Metrics:\n');
    fprintf('--------------------\n');
    fprintf('Peak Lateral G:        %6.2f g\n', results.skidpad.lateral_g);
    fprintf('Average Lap Speed:     %6.2f m/s\n', mean(results.lap.speed_vec));
    fprintf('Maximum Speed:         %6.2f m/s\n', max(results.lap.speed_vec));
    
    fprintf('\nCompetition Points:\n');
    fprintf('-------------------\n');
    fprintf('Acceleration:          %6.1f / 75.5\n', results.points.accel);
    fprintf('Skidpad:               %6.1f / 75.0\n', results.points.skidpad);
    fprintf('Autocross:             %6.1f / 100.0\n', results.points.autocross);
    fprintf('Endurance:             %6.1f / 325.0\n', results.points.endurance);
    fprintf('------------------------------------\n');
    fprintf('TOTAL POINTS:          %6.1f / 575.5\n', results.points.total);
    
    return;
end

%% HELPER FUNCTION
function result = round2(x, y)
    % Round x to nearest multiple of y
    if nargin < 2
        error('round2 requires two arguments');
    end
    
    if numel(y) > 1
        error('Y must be scalar');
    end
    
    result = round(x / y) * y;
    
    return;
end

%% RUN THE SIMULATION
% Execute the main function when script is run
LTS_Consolidated_Complete();