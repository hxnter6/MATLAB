function simReport = LTS_Standalone(opts)
% LTS_Standalone.m
% Self-contained, single-file MATLAB implementation of the LTS Coats lap time simulator.
% - No external scripts required. All helpers embedded as local functions.
% - Produces autocross lap, accel, and skidpad timing similar to original.
% - Fixes several issues: robustness in Straight2, removal of deprecated nargchk,
%   consistent use of units, removal of implicit globals, corner_simple name clash.
%
% Usage:
%   simReport = LTS_Standalone();
%   simReport = LTS_Standalone(struct('track','Ice_Cream_Cone_2','plot',true));
%   simReport = LTS_Standalone(struct('drsEnable',true,'plot',false));
%
% Inputs (optional struct 'opts'):
%   .track        - one of: 'Ice_Cream_Cone_2' (default) or 'BuiltInOval'
%   .plot         - true/false, plots summary figures (default true)
%   .drsEnable    - true/false, enable DRS logic (default true)
%   .engineCSV    - path to engine CSV (columns: rpm, hp, ftlb, L/hr, gearing, shift m/s)
%                    If omitted, uses embedded sample motor map similar to Emrax 208 HV
%   .powerCoeff   - scaling of engine map (default 1.0)
%
% Output:
%   simReport (struct) with fields: lap_time, accel_time, skidpad_time,
%   speed_vec, time_vec, drs_map, parameters, etc.

% ------------------------- Options and defaults ---------------------------
if nargin==0
    opts = struct();
end
if ~isfield(opts,'track'),      opts.track = 'Ice_Cream_Cone_2'; end
if ~isfield(opts,'plot'),       opts.plot = true; end
if ~isfield(opts,'drsEnable'),  opts.drsEnable = true; end
if ~isfield(opts,'powerCoeff'), opts.powerCoeff = 1.0; end

% ------------------------- Simulation parameters --------------------------
dt = 0.001;        % s time step
roundoff = 3;      % time rounding digits for segments
v_tol = 0.001;     % m/s velocity fixed point tolerance (cornering iteration)
a_tol = 1e-4;      % g accel convergence tolerance in Vehicle_Sim_nlin

% Tire model parameters (Pacejka '93) - Hoosier R25B 16x7.5x10 @ 12psi
tire_coeff_lat = [1.497564443, -0.002272231, 2.840472825, 70.86957406, 0.222001058, 369690, -3.24e-07, 0.000447167, -0.003834695, 0.002574449, -0.079271384, -6.855006239];
tire_coeff_lon = [1.2309,-0.0027,2.9719,1.1974,0.0596,4.6389e+04,-0.0140,11.4756,253.5530,138.2941,0.0003,-4.5750];
mulat = 0.65; mulon = 0.65;
SA_max = 15; SR_max = 0.25;
SA_res = SA_max/50; SR_res = SR_max/50;
SA_range = 0:SA_res:SA_max;

% Vehicle geometry / masses
m_driver = 180 * 0.453592;   % kg
m_car    = 280 * 0.453592;   % kg
m_accum  = 60  * 0.453592;   % kg
m_DRS    = 1   * 0.453592;   % kg
m_total  = m_car + m_driver + m_accum + m_DRS;
track_width = 47 * 0.0254;   % m
l = 60.25 * 0.0254;          % m wheelbase
cg_h = 11.2 * 0.0254;        % m
wdf = 0.445;                 % front weight distribution
finaldrive = 4;              % ratio
r_tire = 15.657/2 * 0.0254;  % m loaded radius

% Aero base
area = 1.15; rho = 1.204;            % m^2, kg/m^3
Fl_base = 111 * 4.44822;             % N (downforce at ref speed)
Fd_base = 45  * 4.44822;             % N (drag at ref speed)

% DRS tuning and threshold
drs_drag_reduction = 0.30;           % drag reduction fraction
drs_downforce_loss = 0.10;           % downforce loss fraction
g_thresh_g = 0.90; g_thresh = g_thresh_g * 9.81; % m/s^2

% Velocity range used by vehicle model
v_min = 5.25; v_max = 35.0; dv = 0.05;
v_range = v_min:dv:v_max;

% Engine & shifting
power_coeff = opts.powerCoeff; shift_time = 0.1; rpm_limit = 5500;

% --------------------------- Build track ----------------------------------
[r, d, distance_total] = buildTrack(opts.track);

% ----------------------- Precompute DRS ON/OFF ----------------------------
fprintf('Precomputing vehicle models (DRS %s)...\n', ternary(opts.drsEnable,'enabled','disabled'));

% DRS ON
Fd = Fd_base * (1 - (opts.drsEnable * drs_drag_reduction));
Fl = Fl_base * (1 - (opts.drsEnable * drs_downforce_loss));
[Ay_on, Ax_drive_on, Ax_brake_on, alpha_out_on, c_drag_on, c_lift_on] = ...
    buildVehicleModel(v_range, l, wdf, cg_h, track_width, SA_max, SA_res, SR_max, SR_res, SA_range, ...
                      m_total, area, rho, Fl, Fd, a_tol, tire_coeff_lat, tire_coeff_lon, mulat, mulon, ...
                      finaldrive, r_tire, rpm_limit, power_coeff, []);

% DRS OFF (baseline aero)
Fd = Fd_base; Fl = Fl_base;
[Ay_off, Ax_drive_off, Ax_brake_off, alpha_out_off, c_drag_off, c_lift_off] = ...
    buildVehicleModel(v_range, l, wdf, cg_h, track_width, SA_max, SA_res, SR_max, SR_res, SA_range, ...
                      m_total, area, rho, Fl, Fd, a_tol, tire_coeff_lat, tire_coeff_lon, mulat, mulon, ...
                      finaldrive, r_tire, rpm_limit, power_coeff, []);

% --------------------------- Engine curves --------------------------------
if isfield(opts,'engineCSV') && ~isempty(opts.engineCSV)
    [Fx_engine, fuel_flow, gear_pos] = engineCurvesFromCSV(opts.engineCSV, v_range, finaldrive, r_tire, rpm_limit, power_coeff);
else
    [Fx_engine, fuel_flow, gear_pos] = builtInMotorCurves(v_range, finaldrive, r_tire, rpm_limit, power_coeff);
end

% Attach engine curve into Ax_drive fitters by overriding in the on/off models
% The original code uses Engine_Force_Curves inside Vehicle_Sim_nlin loop; here
% we approximate by limiting Ax_drive by available engine force where it is lower.
Ax_drive_on = @(v) min(Ax_drive_on(v), interp1_clamp(v_range, Fx_engine./m_total, v));
Ax_drive_off = @(v) min(Ax_drive_off(v), interp1_clamp(v_range, Fx_engine./m_total, v));

% -------------------------- Simulate segments -----------------------------
fprintf('Simulating lap...\n');
numSegments = numel(d);
time_out = zeros(numSegments, 20000);
v_out = zeros(numSegments, 20000);
throttle_out = zeros(numSegments, 20000);
drs_active_for_segment = ones(1,numSegments);

lift_on  = @(v) 0.5*c_lift_on * rho * area .* v.^2;
lift_off = @(v) 0.5*c_lift_off * rho * area .* v.^2;

lap_total_time = 0;

for i = 1:numSegments
    seg_r = r(i); seg_d = d(i);
    if seg_r > 0
        % Corner: try DRS ON first
        v_check = 0; corner_conv = false; Ay_try = Ay_on; useLift = lift_on; useDRS = 1;
        while ~corner_conv
            [v_c, time_c, throttle_c] = corner_segment(Ay_try(v_check), seg_r, seg_d, roundoff, dt, 0);
            v_i = mean(v_c);
            if abs(v_check - v_i) >= v_tol
                v_check = abs(v_check + v_i)/2;
            else
                corner_conv = true;
            end
        end
        ay_actual = (v_i^2)/seg_r; % m/s^2
        % Check tire capacity with ON aero; if insufficient, switch to OFF
        Fy_cap_on = sum(computeWheelFyCapacity(ay_actual, v_i, m_total, cg_h, track_width, wdf, SA_max, SA_res, SA_range, tire_coeff_lat, mulat, rho, area, c_lift_on));
        if Fy_cap_on < m_total * ay_actual
            Ay_try = Ay_off; useLift = lift_off; useDRS = 0; v_check = 0; corner_conv = false;
            while ~corner_conv
                [v_c, time_c, throttle_c] = corner_segment(Ay_try(v_check), seg_r, seg_d, roundoff, dt, 0);
                v_i = mean(v_c);
                if abs(v_check - v_i) >= v_tol
                    v_check = abs(v_check + v_i)/2;
                else
                    corner_conv = true;
                end
            end
        end
        % Store
        n = numel(time_c);
        time_out(i,1:n) = time_c;
        v_out(i,1:n) = min_array(v_c, v_max);
        throttle_out(i,1:n) = throttle_c;
        drs_active_for_segment(i) = useDRS;
        lap_total_time = lap_total_time + max(time_c);
    else
        % Straight: choose ON or OFF based on proximity to next corner needs (simple: ON)
        Ax_use = Ax_drive_on; Br_use = Ax_brake_on; useDRS = 1;
        % Estimate entry/exit using neighboring corners
        v_exit = estimateCornerSpeed(i-1, r, d, Ay_on, roundoff, dt, v_tol);
        v_entry = estimateCornerSpeed(i+1, r, d, Ay_on, roundoff, dt, v_tol);
        [time_s, v_s, throttle_s] = straight_segment(seg_d, dt, Ax_use, Br_use, v_exit, v_entry, v_tol);
        n = numel(time_s);
        time_out(i,1:n) = time_s;
        v_out(i,1:n) = v_s;
        throttle_out(i,1:n) = throttle_s;
        drs_active_for_segment(i) = useDRS;
        lap_total_time = lap_total_time + max(time_s);
    end
end

fprintf('Lap time = %.3f s\n', lap_total_time);

% ------------------------ Accel and Skidpad -------------------------------
accel_time = simulateAccel(dt, shift_time, Ax_drive_on, v_range);
skidpad_time = simulateSkidpad(track_width, dt, roundoff, v_tol, Ay_on);

% ------------------------ Post processing ---------------------------------
[time_vec, speed_vec, throttle_vec] = flattenTimeSeries(time_out, v_out, throttle_out);

% ------------------------ Plots -------------------------------------------
if opts.plot
    figure('Name','Time vs Speed'); scatter(time_vec, speed_vec, 6, throttle_vec,'filled'); grid on; colorbar; xlabel('Time (s)'); ylabel('Speed (m/s)'); title(sprintf('Lap time = %.3f s', lap_total_time));
    figure('Name','DRS activation per segment'); bar(drs_active_for_segment); grid on; xlabel('Segment'); ylabel('DRS active (1=on)');
    figure('Name','Slip angle LF vs Speed'); hold on; plot(v_range, alpha_out_on(:,1),'--'); plot(v_range, alpha_out_off(:,1),'-'); legend('ON','OFF'); xlabel('Speed (m/s)'); ylabel('Slip angle (deg)'); grid on;
end

% ------------------------ Output report -----------------------------------
simReport = struct();
simReport.lap_time = lap_total_time;
simReport.accel_time = accel_time;
simReport.skidpad_time = skidpad_time;
simReport.distance_total = distance_total;
simReport.time_vec = time_vec;
simReport.speed_vec = speed_vec;
simReport.throttle_vec = throttle_vec;
simReport.drs_map = drs_active_for_segment;
simReport.parameters = struct('m_total',m_total,'track_width',track_width,'l',l,'cg_h',cg_h,'wdf',wdf,'v_range',[v_min v_max dv]);

end % LTS_Standalone

% =========================== Local functions ==============================
function [r, d, distance_total] = buildTrack(name)
    switch lower(string(name))
        case "ice_cream_cone_2"
            % Simple mixed track: straight-corner-repeat
            % Use representative values similar to repo tracks
            r = [0 15 0 12 0 10 0 9 0 0 8 0 7 0 0 0 0 0];
            d = [40 25 60 20 50 15 60 10 90 50 25 60 20 50 30 40 50 60];
        case "builtinoval"
            r = [20 0 20 0];
            d = [50 50 50 50];
        otherwise
            error('Unknown track "%s"', name);
    end
    distance_total = sum(d);
end

function [Ay, Ax_drive, Ax_brake, alpha_out, c_drag, c_lift] = buildVehicleModel(v_range, l, wdf, cg_h, track, SA_max, SA_res, SR_max, SR_res, SA_range, m_total, area, rho, Fl, Fd, a_tol, tire_coeff_lat, tire_coeff_lon, mulat, mulon, finaldrive, r_tire, rpm_limit, power_coeff, Fx_engine_override)
    g = 9.81;
    % Lift and drag coefficients referenced at 15.6464 m/s (per original code)
    c_lift = 2*Fl/(rho*area*15.6464^2);
    c_drag = 2*Fd/(rho*area*15.6464^2);
    lift = @(v) 0.5*c_lift*rho*area.*v.^2;
    drag = @(v) 0.5*c_drag*rho*area.*v.^2;

    emptygrid = zeros(1,numel(v_range));
    Ay_out = emptygrid; Ax_drive_out = emptygrid; Ax_brake_out = emptygrid;
    alpha_out = zeros(numel(v_range),4);

    % Engine map override (optional)
    builtFxPerV = [];
    if ~isempty(Fx_engine_override)
        builtFxPerV = Fx_engine_override ./ m_total;
    end

    for ii = 1:numel(v_range)
        v = v_range(ii);
        fdowns = lift(v); fdrag = drag(v);

        % Static corner weights (N)
        w_1 = 0.5*m_total*g*wdf;
        w_2 = 0.5*m_total*g*wdf;
        w_3 = 0.5*m_total*g*(1-wdf);
        w_4 = 0.5*m_total*g*(1-wdf);

        % Lateral limit iteration
        Ay_in = 0; Ay_last = 10;
        while abs(Ay_in - Ay_last) >= a_tol
            Ay_last = Ay_in;

            LT_f = Ay_last*m_total*g*cg_h*wdf/track;
            LT_r = Ay_last*m_total*g*cg_h*(1-wdf)/track;

            R1 = (v^2)/(Ay_last*g) - track/2;
            R2 = (v^2)/(Ay_last*g) + track/2;
            R3 = (v^2)/(Ay_last*g) - track/2;
            R4 = (v^2)/(Ay_last*g) + track/2;

            heading1 = atand(l*(1-wdf)/(R1));
            heading2 = atand(l*(1-wdf)/(R2));
            heading3 = atand(l*(wdf)/(R3));
            heading4 = atand(l*(wdf)/(R4));

            SA_delta_f = heading1 - heading2;
            SA_delta_r = heading3 - heading4;

            Fz_1 = (w_1 + 0.5*fdowns*wdf - LT_f)/4.44822;
            Fz_2 = (w_2 + 0.5*fdowns*wdf + LT_f)/4.44822;
            Fz_3 = (w_3 + 0.5*fdowns*(1-wdf) - LT_r)/4.44822;
            Fz_4 = (w_4 + 0.5*fdowns*(1-wdf) + LT_r)/4.44822;

            % Prevent negative normal forces by redistributing (match original behavior)
            if (Fz_1 + Fz_3) < 0, error('CG too high or track too narrow'); end
            if Fz_1 < 0, Fz_2 = Fz_2 - Fz_1; Fz_1 = 0; end
            if Fz_2 < 0, error('Fz_2 < 0'); end
            if Fz_3 < 0, Fz_4 = Fz_4 - Fz_3; Fz_3 = 0; end
            if Fz_4 < 0, error('Fz_4 < 0'); end

            [Fy_1,alpha_1] = Fy_max_local(Fz_1,SA_max,SA_res,tire_coeff_lat,mulat);
            [Fy_2,alpha_2] = Fy_max_local(Fz_2,SA_max,SA_res,tire_coeff_lat,mulat);
            Fy_range_3 = Fy_range_local(Fz_3,SA_range+SA_delta_r,tire_coeff_lat,mulat);
            Fy_range_4 = Fy_range_local(Fz_4,SA_range,tire_coeff_lat,mulat);
            [Fy_rear,SA_r] = max(Fy_range_3+Fy_range_4); %#ok<ASGLU>
            Fy_3 = Fy_range_3(SA_r); Fy_4 = Fy_range_4(SA_r);
            alpha_3 = SA_range(SA_r)+SA_delta_r; alpha_4 = SA_range(SA_r);

            Mz_cg_F = (Fy_1+Fy_2)*(1-wdf);
            Mz_cg_R = (Fy_3+Fy_4)*(wdf);

            if Mz_cg_F > Mz_cg_R
                Mz_cg_F = Mz_cg_R; Fy_F = Mz_cg_F/(1-wdf);
                Fy_F_scale = Fy_F/max(Fy_1 + Fy_2, eps);
                Fy_R_scale = 1;
            elseif Mz_cg_F < Mz_cg_R
                Mz_cg_R = Mz_cg_F; Fy_R = Mz_cg_R/(wdf);
                Fy_R_scale = Fy_R/max(Fy_3 + Fy_4, eps);
                Fy_F_scale = 1;
            else
                Fy_F_scale = 1; Fy_R_scale = 1;
            end

            Ay_in = (Fy_1*Fy_F_scale*4.44822+Fy_2*Fy_F_scale*4.44822+Fy_3*Fy_R_scale*4.44822+Fy_4*Fy_R_scale*4.44822)/(m_total*g);
        end
        alpha_out(ii,:) = [alpha_1 alpha_2 alpha_3 alpha_4];
        Ay_out(ii) = Ay_in*g; % convert back to m/s^2

        % Longitudinal drive limit
        Ax_drive_in = 0; Ax_last = 10;
        while abs(Ax_drive_in - Ax_last) > a_tol
            Ax_last = Ax_drive_in;
            LT = g*m_total*Ax_last*cg_h/(l*2);
            Fz_3 = (w_3 + 0.5*fdowns*(1-wdf) + LT)/4.44822;
            Fz_4 = (w_4 + 0.5*fdowns*(1-wdf) + LT)/4.44822;
            Fx_3 = Fx_drive_local(Fz_3,SR_max,SR_res,tire_coeff_lon,mulon);
            Fx_4 = Fx_drive_local(Fz_4,SR_max,SR_res,tire_coeff_lon,mulon);
            Fx_tire = (Fx_3+Fx_4)*4.44822; % N
            if ~isempty(builtFxPerV)
                Fx_engine_N = builtFxPerV(ii) * m_total;
            else
                Fx_engine_N = inf; % will be limited later after function creation
            end
            if Fx_tire >= Fx_engine_N
                Ax_drive_in = (Fx_engine_N - fdrag)/max(m_total, eps)/g;
            else
                Ax_drive_in = (Fx_tire - fdrag)/max(m_total, eps)/g;
            end
        end
        Ax_drive_out(ii) = Ax_drive_in*g;

        % Longitudinal brake limit
        Ax_brake_in = 0; Ax_last = 10;
        while abs(Ax_brake_in - Ax_last) > a_tol
            Ax_last = Ax_brake_in;
            LT = g*m_total*Ax_brake_in*cg_h/(l*2);
            Fz_1 = (w_1 + 0.5*fdowns*wdf - LT)/4.44822;
            Fz_2 = (w_2 + 0.5*fdowns*wdf - LT)/4.44822;
            Fz_3 = (w_3 + 0.5*fdowns*(1-wdf) + LT)/4.44822;
            Fz_4 = (w_4 + 0.5*fdowns*(1-wdf) + LT)/4.44822;
            Fx_1 = -Fx_brake_local(Fz_1,SR_max,SR_res,tire_coeff_lon,mulon);
            Fx_2 = -Fx_brake_local(Fz_2,SR_max,SR_res,tire_coeff_lon,mulon);
            Fx_3 = -Fx_brake_local(Fz_3,SR_max,SR_res,tire_coeff_lon,mulon);
            Fx_4 = -Fx_brake_local(Fz_4,SR_max,SR_res,tire_coeff_lon,mulon);
            Fx_tire = (Fx_1+Fx_2+Fx_3+Fx_4)*4.44822;
            Ax_brake_in = (Fx_tire - fdrag)/max(m_total, eps)/g;
        end
        Ax_brake_out(ii) = Ax_brake_in*g;
    end

    % Fit polynomials (match original orders)
    P1 = polyfit(v_range, Ay_out, 3);
    P2 = polyfit(v_range, Ax_drive_out, 10);
    P3 = polyfit(v_range, Ax_brake_out, 3);
    Ay       = @(x) polyval(P1, x);
    Ax_drive = @(x) polyval(P2, x);
    Ax_brake = @(x) polyval(P3, x);
end

function [Fx_engine, fuel_flow, gear_pos] = engineCurvesFromCSV(csvPath, v_range, finaldrive, r_tire, rpm_limit, power_coeff)
    % Engine CSV columns: rpm, hp, ftlb, L/hr, gearing, shift m/s (some rows zero)
    try
        values = csvread(csvPath,3,0);
    catch
        error('Failed to read engine CSV: %s', csvPath);
    end
    engine_spd = values(:,1)'; power_hp = values(:,2)'; torque_ftlb = values(:,3)'; fuel_Lhr = values(:,4)';
    gearing_col = values(:,5)'; shifting_ms = values(:,6)';
    gearing = gearing_col(gearing_col~=0); shifting_ms = shifting_ms(shifting_ms~=0);
    gearnum = max(numel(gearing)-1, 1);
    primary = gearing(1);
    torque_Nm = torque_ftlb .* 1.35582 * power_coeff;
    fuel_Ls = fuel_Lhr / 3600;
    tol = engine_spd(1) - engine_spd(2);

    Fx_engine = zeros(size(v_range)); fuel_flow = zeros(size(v_range)); gear_pos = zeros(size(v_range));
    curr_gear = 1;
    for ii = 1:numel(v_range)
        v = v_range(ii);
        if gearnum == 1
            gear_pos(ii) = curr_gear;
            rpm = round2_local((v*primary*gearing(gear_pos(ii)+1)*finaldrive*(60/(2*pi))/r_tire), tol);
            ind = find(engine_spd <= rpm, 1);
            if isempty(ind), ind = numel(engine_spd); end
            fuel_flow(ii) = fuel_Ls(ind);
            Fx_engine(ii) = torque_Nm(ind)*primary*gearing(gear_pos(ii)+1)*finaldrive/r_tire;
        else
            if curr_gear <= numel(shifting_ms) && v > shifting_ms(curr_gear)
                curr_gear = min(curr_gear+1, gearnum);
            end
            gear_pos(ii) = curr_gear;
            rpm = round2_local((v*primary*gearing(gear_pos(ii)+1)*finaldrive*(60/(2*pi))/r_tire), tol);
            ind = find(engine_spd <= rpm, 1);
            if isempty(ind), ind = numel(engine_spd); end
            fuel_flow(ii) = fuel_Ls(ind);
            Fx_engine(ii) = torque_Nm(ind)*primary*gearing(gear_pos(ii)+1)*finaldrive/r_tire;
        end
    end
    % RPM limiter
    for ii = 1:numel(v_range)
        rpm = (v_range(ii)*primary*gearing(min(gear_pos(ii)+1, numel(gearing)))*finaldrive*(60/(2*pi))/r_tire);
        if rpm > rpm_limit, Fx_engine(ii) = 0; end
    end
end

function [Fx_engine, fuel_flow, gear_pos] = builtInMotorCurves(v_range, finaldrive, r_tire, rpm_limit, power_coeff)
    % Built-in simple motor curve resembling an Emrax HV: flat torque then power limited
    % Define a synthetic RPM axis and torque/power
    rpm_axis = 1000:200:7000;
    torque_ftlb = [110*ones(1,15), linspace(110,30,16)]; torque_ftlb = torque_ftlb(1:numel(rpm_axis));
    power_hp = (torque_ftlb .* rpm_axis) / 5252;
    fuel_Lhr = zeros(size(rpm_axis));
    primary = 1.0; gearing = [primary, 3.0, 2.0, 1.5, 1.2]; gearnum = numel(gearing)-1;
    shifting_ms = [10, 20, 28, 33];

    % Build a temporary CSV-like map in memory via interpolation during loop
    torque_Nm_axis = torque_ftlb .* 1.35582 * power_coeff;
    Fx_engine = zeros(size(v_range)); fuel_flow = zeros(size(v_range)); gear_pos = zeros(size(v_range));
    curr_gear = 1;
    for ii = 1:numel(v_range)
        v = v_range(ii);
        if curr_gear <= numel(shifting_ms) && v > shifting_ms(curr_gear)
            curr_gear = min(curr_gear+1, gearnum);
        end
        gear_pos(ii) = curr_gear;
        rpm = (v*primary*gearing(curr_gear+1)*finaldrive*(60/(2*pi))/r_tire);
        rpm = max(min(rpm, max(rpm_axis)), min(rpm_axis));
        tq = interp1(rpm_axis, torque_Nm_axis, rpm, 'linear');
        Fx_engine(ii) = tq*primary*gearing(curr_gear+1)*finaldrive/r_tire;
        if rpm > rpm_limit, Fx_engine(ii) = 0; end
    end
end

function [v_, t_, throttle] = corner_segment(Ay, r, d, roundoff, dt, t_start)
    v = sqrt(max(Ay,0) * max(r,eps));
    t = d/max(v, eps);
    t = round(t, roundoff);
    t_end = t_start + t;
    t_ = t_start:dt:t_end;
    if isempty(t_), t_ = 0; end
    v_ = ones(1, numel(t_)) .* v;
    throttle = 0.3 .* ones(1, numel(t_));
end

function [Fy_max_N, Fz_N] = computeWheelFyCapacity(ay_trial, v_guess, m_total, cg_h, track_width, wdf, SA_max, SA_res, SA_range, tire_coeff_lat, mulat, rho, area, c_lift)
    % Compute per-wheel Fy capacity given lateral acceleration using Pacejka
    W_total_N = m_total*9.81;
    lift_v = 0.5*c_lift*rho*area*(v_guess^2);
    DeltaF_lat_total = m_total * ay_trial * cg_h / max(track_width, eps);
    LT_front = DeltaF_lat_total * wdf;
    LT_rear  = DeltaF_lat_total * (1-wdf);
    Fz_front_per_wheel = (W_total_N + lift_v) * wdf / 2;
    Fz_rear_per_wheel  = (W_total_N + lift_v) * (1-wdf) / 2;
    Fz_LF = Fz_front_per_wheel + 0.5*LT_front;
    Fz_RF = Fz_front_per_wheel - 0.5*LT_front;
    Fz_LR = Fz_rear_per_wheel  + 0.5*LT_rear;
    Fz_RR = Fz_rear_per_wheel  - 0.5*LT_rear;
    Fz_N = max([Fz_LF;Fz_RF;Fz_LR;Fz_RR], 0);
    Fz_lbf = Fz_N / 4.44822;
    Fy_lbf = zeros(4,1);
    for kk = 1:4
        [Fy_tmp, ~] = Fy_max_local(Fz_lbf(kk), SA_max, SA_res, tire_coeff_lat, mulat);
        Fy_lbf(kk) = Fy_tmp;
    end
    Fy_max_N = Fy_lbf * 4.44822;
end

function [time, velocity, throttle] = straight_segment(dist, dt, Ax_drive, Ax_brake, v_initial, v_final, vtol)
    d = dist;
    t_d = 0; x_drive = 0; xd_drive = v_initial; xdd_drive = 0;
    drive_t = t_d; drive_v = xd_drive; drive_x = x_drive; drive_a = xdd_drive;
    % Forward integrate (drive)
    max_steps = max(10, ceil(d/(max(v_initial,1)*dt)) + 100000);
    for j = 1:max_steps
        t_next = drive_t(end) + dt;
        a_next = Ax_drive(drive_v(end));
        v_next = drive_v(end) + 0.5*(a_next + drive_a(end))*dt;
        x_next = drive_x(end) + 0.5*(v_next + drive_v(end))*dt;
        drive_t(end+1) = t_next; %#ok<AGROW>
        drive_a(end+1) = a_next; %#ok<AGROW>
        drive_v(end+1) = v_next; %#ok<AGROW>
        drive_x(end+1) = x_next; %#ok<AGROW>
        if x_next >= d, break; end
    end
    % Backward integrate (brake)
    t_b = drive_t(end); x_brake = d; xd_brake = v_final; xdd_brake = 0;
    brake_t = t_b; brake_v = xd_brake; brake_x = x_brake; brake_a = xdd_brake;
    for k = 1:max_steps
        t_prev = brake_t(end) - dt;
        a_next = -Ax_brake(brake_v(end));
        v_next = brake_v(end) + 0.5*(a_next + brake_a(end))*dt;
        x_next = brake_x(end) - 0.5*(v_next + brake_v(end))*dt;
        brake_t(end+1) = t_prev; %#ok<AGROW>
        brake_a(end+1) = a_next; %#ok<AGROW>
        brake_v(end+1) = v_next; %#ok<AGROW>
        brake_x(end+1) = x_next; %#ok<AGROW>
        if x_next <= 0, break; end
    end
    % Align arrays
    N = min(numel(drive_v), numel(brake_v));
    drive_v = drive_v(1:N); brake_v = fliplr(brake_v(1:N));
    throttle = zeros(1,N); velocity = zeros(1,N); time = zeros(1,N);
    time(1) = 0;
    for i = 1:N-1
        if drive_v(i) >= brake_v(i)
            throttle(i) = 0; velocity(i) = brake_v(i);
        else
            throttle(i) = 1; velocity(i) = drive_v(i);
        end
        time(i+1) = time(i) + dt;
    end
end

function accel_time = simulateAccel(dt, shift_time, Ax_drive, v_range)
    acceltime = 0; w = 1; d_accel = 0; xd_accel = 5; xdd_accel = 0;
    while d_accel <= 76
        acceltime = acceltime + dt;
        xdd_accel = abs(Ax_drive(xd_accel));
        xd_accel = xd_accel + 0.5*dt*(xdd_accel + xdd_accel);
        d_accel = d_accel + 0.5*dt*(xd_accel + xd_accel);
        w = w+1; %#ok<NASGU>
        if acceltime > 200, error('Accel sim divergence'); end
    end
    % no real shifting without gear schedule; apply small constant penalty to mimic original behavior
    acceltime = acceltime + shift_time; 
end

function skidpad_time = simulateSkidpad(track, dt, roundoff, v_tol, Ay)
    % Skidpad uses r = 7.625 + track/2 and finds consistent Ay speed
    r_skid = 7.625 + track/2;
    v_check = 0; corner_conv = false;
    while ~corner_conv
        [v_skid, time_skid] = corner_segment(Ay(v_check), r_skid, r_skid*2*pi, roundoff, dt, 0);
        v_i = mean(v_skid);
        if abs(v_check - v_i) >= v_tol
            v_check = abs(v_check + v_i)/2;
        else
            corner_conv = true;
        end
    end
    skidpad_time = max(time_skid);
end

function [time_vec, speed_vec, throttle_vec] = flattenTimeSeries(time_out, v_out, throttle_out)
    [rows, cols] = size(time_out);
    time_vec = []; speed_vec = []; throttle_vec = [];
    ref = 0;
    for i = 1:rows
        for j = 1:cols
            t = time_out(i,j);
            if t ~= 0
                time_vec(end+1) = t + ref; %#ok<AGROW>
                speed_vec(end+1) = v_out(i,j); %#ok<AGROW>
                throttle_vec(end+1) = throttle_out(i,j); %#ok<AGROW>
            end
        end
        ref = ref + max(time_out(i,:));
    end
    if ~isempty(speed_vec)
        speed_vec(1) = speed_vec(min(2, numel(speed_vec)));
    end
end

function out = ternary(b, a_true, a_false)
    if b, out = a_true; else, out = a_false; end
end

function x = min_array(a, maxv)
    x = a; x(a>maxv) = maxv;
end

function y = interp1_clamp(x, v, xi)
    if xi <= x(1), y = v(1); elseif xi >= x(end), y = v(end); else, y = interp1(x, v, xi, 'linear'); end
end

% ---- Local tire helper variants (no globals) ----
function [ max_Fy,alpha ] = Fy_max_local( Fz,SA_max,res, tire_coeff_lat, mulat )
    SA = 0:res:SA_max; Fy = zeros(size(SA));
    for i = 1:numel(SA)
        Fy(i) = pacejka_fun_93_local(tire_coeff_lat,[Fz SA(i) mulat]);
    end
    [max_Fy,I] = max(Fy); alpha = SA(I);
end

function [ Fy ] = Fy_range_local( Fz,SA_range, tire_coeff_lat, mulat )
    Fy = zeros(size(SA_range));
    for i = 1:numel(SA_range)
        Fy(i) = pacejka_fun_93_local(tire_coeff_lat,[Fz SA_range(i) mulat]);
    end
end

function [ max_Fx ] = Fx_drive_local( Fz,SR_max,res, tire_coeff_lon, mulon )
    SR = 0:res:SR_max; Fx = zeros(size(SR));
    for i = 1:numel(SR)
        Fx(i) = pacejka_fun_93_local(tire_coeff_lon,[Fz SR(i) mulon]);
    end
    max_Fx = max(Fx);
end

function [ max_Fx ] = Fx_brake_local( Fz,SR_max,res, tire_coeff_lon, mulon )
    SR = 0:-res:-SR_max; Fx = zeros(size(SR));
    for i = 1:numel(SR)
        Fx(i) = pacejka_fun_93_local(tire_coeff_lon,[Fz SR(i) mulon]);
    end
    max_Fx = -min(Fx);
end

function Y = pacejka_fun_93_local(beta,in)
    Fz = in(1); X = in(2); mu = in(3);
    C = beta(1); c1 = beta(2); c2 = beta(3); c3 = beta(4); c4 = beta(5); c5 = beta(6); c6 = beta(7); c7 = beta(8); c8 = beta(9); dE = beta(10); SH = beta(11); SV = beta(12);
    D = mu.*(c1*Fz.^2+c2*Fz);
    BCD = c3*sind(c4*atand(c5*Fz));
    B = BCD./(C.*D + eps);
    x = X + SH;
    E = (c6*Fz.^2+c7.*Fz+c8)+dE*sign(x);
    y = D.*sind(C.*atand(B.*x-E.*(B.*x-atand(B.*x))));
    Y0 = y + SV; Y0(isnan(Y0)) = 0; Y = Y0;
end

function z = round2_local(x,y)
    if ~isscalar(y)
        error('Y must be scalar');
    end
    z = round(x/y)*y;
end

function v_est = estimateCornerSpeed(idx, r, d, Ay, roundoff, dt, v_tol)
    n = numel(d);
    if idx < 1, idx = n; end
    if idx > n, idx = 1; end
    if r(idx) <= 0
        v_est = 10; % default for straights if neighbor is straight
        return;
    end
    v_check = 0; corner_conv = false;
    while ~corner_conv
        v_c = corner_segment(Ay(v_check), r(idx), d(idx), roundoff, dt, 0);
        if iscell(v_c)
            v_mean = mean(v_c{1});
        else
            v_mean = mean(v_c);
        end
        if abs(v_check - v_mean) >= v_tol
            v_check = abs(v_check + v_mean)/2;
        else
            corner_conv = true;
        end
    end
    if iscell(v_c)
        v_est = mean(v_c{1});
    else
        v_est = mean(v_c);
    end
end

