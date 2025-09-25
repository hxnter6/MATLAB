%% run_lts_sim_with_drs.m
% Consolidated runner that precomputes DRS ON/OFF vehicle models and runs a lap,
% disabling DRS on corners when lateral G exceeds threshold.
% Requires your Vehicle_Sim_nlin, Engine_Force_Curves, corner, Straight2, Fy_max, and track file.

clearvars -except simReport; close all; clc
fprintf('Running LTS lap sim with DRS auto-disable logic...\n');

%% 1) Track: choose Ice Cream Cone 2 (must exist in Tracks folder)
trackScript = 'Ice_Cream_Cone_2'; % change if needed
if exist([trackScript '.m'],'file') ~= 2
    error('Track file %s.m not found in current folder. Place it or change trackScript.', trackScript);
end
run(trackScript); % must define r, d, distance_total
if ~exist('r','var') || ~exist('d','var')
    error('Track script did not produce r and d arrays.');
end
numSegments = numel(d);

%% 2) Simulation params (from your provided LTS_TUI values, cleaned)
dt = 0.001;
roundoff = 3;
v_tol = 0.001;
a_tol = 1e-4;
v_min = 5.25; v_max = 35.0; dv = 0.05;
v_range = v_min:dv:v_max;

% Tire model / params (from your file)
SA_max = 15; SR_max = 0.25;
SA_res = SA_max/50; SR_res = SR_max/50;
SA_range = 0:SA_res:SA_max;

% vehicle geometry & masses (your numbers)
m_driver = 180 * 0.453592;
m_car = 280 * 0.453592;
m_accum = 60 * 0.453592;
m_DRS = 1 * 0.453592;
m_total = m_car + m_driver + m_DRS + m_accum; % don't use mass hack
track_width = 47 * 0.0254; % your 'track' var
l = 60.25 * 0.0254; cg_h = 11.2 * 0.0254;
wdf = 0.445; finaldrive = 4;
r_tire = 15.657/2 * 0.0254;

% Aero base (your numbers)
area = 1.15; rho = 1.204;
Fl_base = 111 * 4.44822;
Fd_base = 45 * 4.44822;

% DRS tuning (tweak as needed)
drs_drag_reduction = 0.30;   % fractional drag reduction when DRS active
drs_downforce_loss = 0.10;   % fractional downforce loss when DRS active

% Lateral-G cutoff
g_thresh_g = 0.90; % [g] threshold to force DRS off in corners
g_thresh = g_thresh_g * 9.81; % [m/s^2]

% other
shift_time = 0.1;
rpm_limit = 5500;

%% 3) Check required helper functions exist
needed = {'Vehicle_Sim_nlin.m','corner.m','Straight2.m','Fy_max.m','pacejka_fun_93.m'};
for k=1:numel(needed)
    if exist(needed{k},'file') ~= 2
        warning('Missing helper: %s. Ensure it exists in path.', needed{k});
    end
end

%% 4) Precompute two vehicle models: DRS ON and DRS OFF
fprintf('Precomputing vehicle models (DRS ON & OFF)...\n');

% Helper to compute function handles by running Vehicle_Sim_nlin script
functionHandles = struct();

for drs_flag = [1 0]
    if drs_flag
        Fd = Fd_base * (1 - drs_drag_reduction);
        Fl = Fl_base * (1 - drs_downforce_loss);
    else
        Fd = Fd_base;
        Fl = Fl_base;
    end
    c_lift = 2*Fl/(rho*area*15.6464^2);
    c_drag = 2*Fd/(rho*area*15.6464^2);
    lift = @(v) 0.5*c_lift*rho*area.*v.^2;
    drag = @(v) 0.5*c_drag*rho*area.*v.^2;

    % Provide lift/drag to Vehicle_Sim_nlin via workspace variables (script reads these)
    % Ensure Vehicle_Sim_nlin uses v_range etc. from workspace
    try
        Vehicle_Sim_nlin; % must result in Ay, Ax_drive, Ax_brake, alpha_out variables
    catch ME
        error('Error running Vehicle_Sim_nlin: %s', ME.message);
    end

    tag = ternary(drs_flag,'on','off');
    functionHandles.(sprintf('Ay_%s',tag)) = Ay;
    functionHandles.(sprintf('Ax_drive_%s',tag)) = Ax_drive;
    functionHandles.(sprintf('Ax_brake_%s',tag)) = Ax_brake;
    functionHandles.(sprintf('alpha_out_%s',tag)) = alpha_out;
    functionHandles.(sprintf('c_drag_%s',tag)) = c_drag;
    functionHandles.(sprintf('c_lift_%s',tag)) = c_lift;
end

% default active handles (start with DRS ON)
Ay_on = functionHandles.Ay_on;
Ax_drive_on = functionHandles.Ax_drive_on;
Ax_brake_on = functionHandles.Ax_brake_on;
Ay_off = functionHandles.Ay_off;
Ax_drive_off = functionHandles.Ax_drive_off;
Ax_brake_off = functionHandles.Ax_brake_off;

fprintf('Precompute done. c_drag_on=%.6f c_drag_off=%.6f\n', functionHandles.c_drag_on, functionHandles.c_drag_off);

%% 5) Run lap: segment by segment (corners use DRS auto-disable)
fprintf('Simulating lap (segments: %d)...\n', numSegments);

% Initialize outputs (sparse preallocation)
maxPointsPerSeg = 200000; % just to be safe
time_out = zeros(numSegments,20000);
v_out = zeros(numSegments,20000);
throttle_out = zeros(numSegments,20000);
manuver_num = zeros(numSegments,20000);
drs_active_for_segment = ones(1,numSegments);

cumDistance = 0;
distance_vec = [];
speed_vec = [];
time_vec = [];
lateralG_vec = [];
drs_map = [];

lap_total_time = 0;

% Helper to compute per-wheel loads & Fy capacity using Pacejka (Fy_max); returns Fy_max per wheel (N) and Fz per wheel (N)
function [Fy_max_N, Fz_N] = compute_wheel_caps_from_ay(ay_trial, v_guess)
    % ay_trial [m/s^2], v_guess used to get aero downforce for lift(v)
    W_total_N = m_total*9.81;
    lift_v = lift(v_guess); % lift function from last Vehicle_Sim_nlin run; must exist in workspace - we will reassign below
    DeltaF_lat_total = m_total * ay_trial * cg_h / track_width;
    LT_front = DeltaF_lat_total * wdf;
    LT_rear  = DeltaF_lat_total * (1-wdf);
    % per-wheel static normal loads
    Fz_front_per_wheel = (W_total_N + lift_v) * wdf / 2;
    Fz_rear_per_wheel  = (W_total_N + lift_v) * (1-wdf) / 2;
    Fz_LF = Fz_front_per_wheel + 0.5*LT_front;
    Fz_RF = Fz_front_per_wheel - 0.5*LT_front;
    Fz_LR = Fz_rear_per_wheel  + 0.5*LT_rear;
    Fz_RR = Fz_rear_per_wheel  - 0.5*LT_rear;
    Fz_N = max([Fz_LF;Fz_RF;Fz_LR;Fz_RR], 0);
    % convert to lbf for Fy_max (your Fy_max expects Fz in lbf)
    Fz_lbf = Fz_N / 4.44822;
    Fy_lbf = zeros(4,1);
    for kk=1:4
        [Fy_tmp, ~] = Fy_max(Fz_lbf(kk), SA_max, SA_res); % returns lbf
        Fy_lbf(kk) = Fy_tmp;
    end
    Fy_max_N = Fy_lbf * 4.44822;
end

% Note: we will use the Ay functions computed earlier, and we must also set lift function used by compute_wheel_caps_from_ay.
% Create two lift functions for on/off so wheel calc uses correct aero
c_lift_on = functionHandles.c_lift_on; c_lift_off = functionHandles.c_lift_off;
lift_on = @(v) 0.5*c_lift_on*rho*area.*v.^2;
lift_off = @(v) 0.5*c_lift_off*rho*area.*v.^2;

% Iterate segments
for i = 1:numSegments
    seg_r = r(i);
    seg_d = d(i);
    if seg_r > 0 % corner
        % Try DRS-ON first
        Ay_try = Ay_on;
        lift = lift_on; % set lift for wheel calc & Vehicle_Sim usage
        v_check = 0; corner_conv = 0;
        while corner_conv ~= 1
            [v_c, time_c, throttle_c] = corner(Ay_try(v_check), seg_r, seg_d, roundoff, dt, 0);
            v_i = mean(v_c);
            if abs(v_check - v_i) >= v_tol
                v_check = abs(v_check + v_i)/2;
            else
                corner_conv = 1;
            end
        end
        % actual lateral acceleration:
        ay_actual = (v_i^2)/seg_r;
        % compute wheel capacities using lift_on at this speed
        lift = lift_on;
        [Fy_cap_on, Fz_on] = compute_wheel_caps_from_ay(ay_actual, v_i);
        % check if capacity sum >= demand
        if sum(Fy_cap_on) < m_total * ay_actual
            % DRS-ON cannot sustain corner; recompute with DRS-OFF
            Ay_try = Ay_off;
            lift = lift_off;
            v_check = 0; corner_conv = 0;
            while corner_conv ~= 1
                [v_c, time_c, throttle_c] = corner(Ay_try(v_check), seg_r, seg_d, roundoff, dt, 0);
                v_i = mean(v_c);
                if abs(v_check - v_i) >= v_tol
                    v_check = abs(v_check + v_i)/2;
                else
                    corner_conv = 1;
                end
            end
            ay_actual = (v_i^2)/seg_r;
            [Fy_cap_off, Fz_off] = compute_wheel_caps_from_ay(ay_actual, v_i);
            drs_active_for_segment(i) = 0;
            usedAx = Ax_drive_off; usedAy = Ay_off; usedLift = lift_off;
        else
            drs_active_for_segment(i) = 1;
            usedAx = Ax_drive_on; usedAy = Ay_on; usedLift = lift_on;
        end

        % store outputs for this corner
        for j = 1:numel(time_c)
            time_out(i,j) = time_c(j);
            v_out(i,j) = min(v_c(j), v_max);
            throttle_out(i,j) = throttle_c(j);
            manuver_num(i,j) = i;
            % append global vectors for plotting
            cumDistance = cumDistance + ( (j==1) * 0 + 0 ); % distance handled later by integration
            distance_vec(end+1) = cumDistance; %#ok<SAGROW>
            speed_vec(end+1) = v_out(i,j); %#ok<SAGROW>
            time_vec(end+1) = time_out(i,j); %#ok<SAGROW>
            lateralG_vec(end+1) = (v_out(i,j)^2)/seg_r / 9.81; %#ok<SAGROW>
            drs_map(end+1) = drs_active_for_segment(i); %#ok<SAGROW>
        end

        lap_total_time = lap_total_time + max(time_c);
    else
        % Straight segment: decide if DRS allowed. We allow DRS by default,
        % but disable it on this straight if the *next* corner requires OFF (optional)
        nextCornerNeedsOff = 0;
        if i < numSegments
            % quick check: if next segment is corner and small radius, precompute
            if r(i+1) > 0
                % Estimate speed if DRS used on straight: compute with Ax_drive_on
                % We'll do a coarse check: compute d->exit speed using Straight2 with Ax_drive_on
                % Use current v_exit estimate from previous segment if needed
                v_guess_entry = v_max/2;
                % For simplicity allow DRS on straights; advanced logic can be added
                nextCornerNeedsOff = 0;
            end
        end

        if nextCornerNeedsOff == 0
            Ax_use = Ax_drive_on; Br_use = Ax_brake_on;
            drs_active_for_segment(i) = 1;
        else
            Ax_use = Ax_drive_off; Br_use = Ax_brake_off;
            drs_active_for_segment(i) = 0;
        end

        % Determine v_entry and v_exit for this straight using existing logic
        % For simplicity assume rolling start: use previous v_i as v_exit (if exist) and next corner's entry velocity
        % We'll estimate v_entry and v_exit with corner() on neighboring segments if available
        % Here use a simple approach: use mean of last corner exit and next corner entry if available
        if i==1
            % first straight: compute entry/exit from neighboring corner (wrap track)
            % Use corner with last segment as exit, next as entry
            try
                v_last = corner(Ay_on(0), r(numSegments), d(numSegments), roundoff, dt, 0);
                v_exit = mean(v_last);
            catch
                v_exit = v_min;
            end
            try
                v_next = corner(Ay_on(0), r(i+1), d(i+1), roundoff, dt, 0);
                v_entry = mean(v_next);
            catch
                v_entry = v_min;
            end
        elseif i==numSegments
            try
                v_last = corner(Ay_on(0), r(i-1), d(i-1), roundoff, dt, 0);
                v_exit = mean(v_last);
            catch
                v_exit = v_min;
            end
            try
                v_next = corner(Ay_on(0), r(1), d(1), roundoff, dt, 0);
                v_entry = mean(v_next);
            catch
                v_entry = v_min;
            end
        else
            try
                v_last = corner(Ay_on(0), r(i-1), d(i-1), roundoff, dt, 0);
                v_exit = mean(v_last);
            catch
                v_exit = v_min;
            end
            try
                v_next = corner(Ay_on(0), r(i+1), d(i+1), roundoff, dt, 0);
                v_entry = mean(v_next);
            catch
                v_entry = v_min;
            end
        end

        [time_s, v_s, throttle_s] = Straight2(seg_d, dt, Ax_use, Br_use, v_exit, v_entry, v_tol);

        for k = 1:numel(time_s)
            time_out(i,k) = time_s(k);
            v_out(i,k) = v_s(k);
            throttle_out(i,k) = throttle_s(k);
            manuver_num(i,k) = i;
            distance_vec(end+1) = cumDistance + (sum(v_s(1:k))*dt); %#ok<SAGROW>
            speed_vec(end+1) = v_s(k); %#ok<SAGROW>
            time_vec(end+1) = time_s(k); %#ok<SAGROW>
            lateralG_vec(end+1) = 0; %#ok<SAGROW>
            drs_map(end+1) = drs_active_for_segment(i); %#ok<SAGROW>
        end

        lap_total_time = lap_total_time + max(time_s);
    end
end

fprintf('Lap simulation complete. Lap time = %.3f s\n', lap_total_time);

%% 6) Post-process & plots
% Make a distance vector by integrating speed*time (approx)
if isempty(distance_vec)
    distance_vec = (0:length(speed_vec)-1) .* (mean(speed_vec)*dt);
end

% Plot speed vs index (proxy for distance)
figure('Name','Speed and lateral G vs sample index','NumberTitle','off');
subplot(3,1,1)
plot(speed_vec); grid on; ylabel('Speed (m/s)'); title('Speed vs sample index');
subplot(3,1,2)
plot(lateralG_vec); grid on; ylabel('Lat G (g)');
subplot(3,1,3)
stairs(drs_map); grid on; ylabel('DRS active (1=yes)'); xlabel('sample index');

% Plot speed vs distance (approx)
figure('Name','Speed vs distance','NumberTitle','off');
plot(distance_vec, speed_vec,'.-'); grid on;
xlabel('Distance (m)'); ylabel('Speed (m/s)'); title(sprintf('Lap speed (lap time = %.3f s)', lap_total_time));

% DRS activation heatmap along track segments
figure('Name','DRS activation per segment','NumberTitle','off');
bar(drs_active_for_segment);
xlabel('Segment'); ylabel('DRS active (1=on)'); title('DRS activation per segment');

% Show slip angles vs speed from precomputed alpha_out for ON/OFF
figure('Name','Slip Angles vs Speed (precomputed)','NumberTitle','off');
hold on;
if isfield(functionHandles,'alpha_out_on')
    plot(v_range, functionHandles.alpha_out_on(:,1),'--');
end
if isfield(functionHandles,'alpha_out_off')
    plot(v_range, functionHandles.alpha_out_off(:,1),'-');
end
legend('LF ON','LF OFF'); xlabel('Speed (m/s)'); ylabel('Slip angle (deg)');
grid on;

% Save report
simReport.lap_time = lap_total_time;
simReport.distance_total = distance_total;
simReport.speed_vec = speed_vec;
simReport.time_vec = time_vec;
simReport.drs_map = drs_map;
simReport.drs_active_for_segment = drs_active_for_segment;

assignin('base','simReport',simReport);
fprintf('Simulation finished. simReport saved to workspace.\n');

%% ======= local helper functions =======
function out = ternary(b,a_true,a_false)
    if b, out=a_true; else out=a_false; end
end
