%% LTS_TUI_fixed.m
% Re-written top-level runner for LTS lap sim with DRS on/off precompute.
% Assumes your helper scripts (Vehicle_Sim_nlin.m, Engine_Force_Curves.m,
% corner.m, Straight2.m, Fy_max.m, pacejka_fun_93.m) exist in path.
%
% Usage: run LTS_TUI_fixed from MATLAB prompt. Edit parameters below as needed.

clearvars -except simReport; close all; clc;
fprintf('LTS_TUI_fixed startup...\n');

%% ----------------------- Simulation / Track selection --------------------
addpath('Tracks');
trackScript = 'Ice_Cream_Cone_2'; % <<-- pick the track script you want to simulate

if exist([trackScript '.m'],'file') ~= 2
    error('Track script "%s.m" not found. Place it into /Tracks or change trackScript.', trackScript);
end
run(trackScript);  % must define r, d, distance_total
if ~exist('r','var') || ~exist('d','var')
    error('Track script did not produce r and d arrays.');
end
fprintf('Loaded track: %s  (segments = %d, total distance %.1f m)\n', trackScript, numel(d), distance_total);

%% ----------------------- Simulation parameters ---------------------------
% time stepping and tolerances
dt = 0.001;
roundoff = 3;
v_tol = 0.001;
a_tol = 1e-4;

% tyre model params (reuse yours)
SA_max = 15; SR_max = 0.25;
SA_res = SA_max/50; SR_res = SR_max/50;
SA_range = 0:SA_res:SA_max;

% vehicle geometry / masses (from your repo)
m_driver = 180 * 0.453592;
m_car    = 280 * 0.453592;
m_accum  = 60  * 0.453592;
m_DRS    = 1   * 0.453592;
m_total  = m_car + m_driver + m_accum + m_DRS;   % don't use "m_drag" hack
track_width = 47 * 0.0254;    % 'track' in your workspace
l = 60.25 * 0.0254;
cg_h = 11.2 * 0.0254;
wdf = 0.445;
finaldrive = 4;
r_tire = 15.657/2 * 0.0254;

% engine & other stuff (keep defaults)
power_coeff = 1;
shift_time = 0.1;
rpm_limit = 5500;

% Aerodynamics base (your numbers)
area = 1.15; rho = 1.204;
Fl_base = 111 * 4.44822;  % downforce @ reference speed (N)
Fd_base = 45  * 4.44822;  % drag @ reference speed (N)

% DRS tuning
drs_drag_reduction = 0.30;   % fractional drag reduction when DRS active (0..1)
drs_downforce_loss = 0.10;   % fractional downforce loss when DRS active (0..1)

% lateral G threshold
g_thresh_g = 0.90;           % threshold in g units to force DRS OFF in corners
g_thresh = g_thresh_g * 9.81;

% velocity range used by Vehicle_Sim_nlin (must match expectations of that script)
v_min = 5.25; v_max = 35.0; dv = 0.05;
v_range = v_min:dv:v_max;

%% ----------------------- Precompute DRS ON and OFF vehicle models ------
fprintf('Precomputing vehicle models (DRS ON and OFF). This runs Vehicle_Sim_nlin twice...\n');

% We will set lift and drag function handles and then run Vehicle_Sim_nlin (script)
% DRS = ON
Fd = Fd_base * (1 - drs_drag_reduction);
Fl = Fl_base * (1 - drs_downforce_loss);
c_lift = 2*Fl/(rho*area*15.6464^2);
c_drag = 2*Fd/(rho*area*15.6464^2);
lift = @(v) 0.5*c_lift*rho*area.*v.^2;
drag = @(v) 0.5*c_drag*rho*area.*v.^2;

% Vehicle_Sim_nlin is a script that expects v_range, lift, drag, etc. in workspace.
try
    Vehicle_Sim_nlin; % populates Ay, Ax_drive, Ax_brake, alpha_out
catch ME
    error('Error running Vehicle_Sim_nlin for DRS ON: %s\nEnsure Vehicle_Sim_nlin.m is in path and uses lift/drag/v_range as workspace variables.', ME.message);
end
Ay_drs_on = Ay; Ax_drive_drs_on = Ax_drive; Ax_brake_drs_on = Ax_brake; alpha_out_on = alpha_out;
c_drag_on = c_drag; c_lift_on = c_lift;

% DRS = OFF
Fd = Fd_base;
Fl = Fl_base;
c_lift = 2*Fl/(rho*area*15.6464^2);
c_drag = 2*Fd/(rho*area*15.6464^2);
lift = @(v) 0.5*c_lift*rho*area.*v.^2;
drag = @(v) 0.5*c_drag*rho*area.*v.^2;

try
    Vehicle_Sim_nlin;
catch ME
    error('Error running Vehicle_Sim_nlin for DRS OFF: %s', ME.message);
end
Ay_drs_off = Ay; Ax_drive_drs_off = Ax_drive; Ax_brake_drs_off = Ax_brake; alpha_out_off = alpha_out;
c_drag_off = c_drag; c_lift_off = c_lift;

fprintf('Done precomputing models. c_drag_on=%.6f c_drag_off=%.6f\n', c_drag_on, c_drag_off);

% Default active handles (you can change in-sim)
Ay_on = Ay_drs_on; Ax_drive_on = Ax_drive_drs_on; Ax_brake_on = Ax_brake_drs_on;
Ay_off = Ay_drs_off; Ax_drive_off = Ax_drive_drs_off; Ax_brake_off = Ax_brake_drs_off;

%% ----------------------- Engine force curves (makes Fx_engine, gear data) -
% Uses Engine_Force_Curves script which expects Enginecode, v_range, etc.
addpath('Engines');
Motor = 'Emrax 208 HV';
Enginecode = [Motor '.csv'];
if exist(Enginecode,'file')~=2
    warning('Engine CSV not found (%s). Ensure Engines/%s exists or modify Enginecode variable.', Enginecode, Enginecode);
end
try
    Engine_Force_Curves; % script: creates Fx_engine, fuel_flow, gear_pos, etc.
catch ME
    warning('Engine_Force_Curves failed: %s (continuing; some features may not work).', ME.message);
end

%% ----------------------- Run Maneuver_Sim_fixed (segment-by-segment) ---
% We call our robust Maneuver_Sim_fixed function (provided below). It returns
% time_out, v_out, throttle_out, drs_active_for_segment, laptime.
fprintf('Running maneuver simulation (fixed)...\n');
[time_out, v_out, throttle_out, drs_active_for_segment, laptime] = ...
    Maneuver_Sim_fixed(r, d, Ay_on, Ay_off, Ax_drive_on, Ax_drive_off, Ax_brake_on, Ax_brake_off, ...
                       g_thresh, v_tol, roundoff, dt, v_max, lift, r_tire, m_total, cg_h, track_width, SA_max, SA_res);

fprintf('\n--- RESULTS ---\nLap time = %.3f s\n', laptime);

%% ----------------------- Post processing & plots -----------------------
% call Post_Processing-like behavior: flatten times into vector and print results
% We produce quick plots: speed vs time, lateral g vs time (calculated), drs map
% Flatten outputs:
time_vec = []; speed_vec = []; throttle_vec = []; seg_idx_vec = []; drs_map = [];
for i = 1:size(time_out,1)
    nonz = find(time_out(i,:)~=0);
    time_vec = [time_vec time_out(i,nonz)];
    speed_vec = [speed_vec v_out(i,nonz)];
    throttle_vec = [throttle_vec throttle_out(i,nonz)];
    seg_idx_vec = [seg_idx_vec i*ones(1,numel(nonz))];
    drs_map = [drs_map drs_active_for_segment(i)*ones(1,numel(nonz))];
end

% time vs speed
figure('Name','Time vs Speed'); scatter(time_vec, speed_vec, 6, throttle_vec,'filled'); colorbar; xlabel('Time (s)'); ylabel('Speed (m/s)'); title('Time vs Speed (throttle color)');

% DRS activation along segment index (bar)
figure('Name','DRS activation per segment'); bar(drs_active_for_segment); xlabel('Segment'); ylabel('DRS active (1=on)'); title('DRS active per segment');

% Slip angle precomputed compare
figure('Name','Slip angle (LF) vs Speed ON/OFF'); hold on;
plot(v_range, alpha_out_on(:,1),'--'); plot(v_range, alpha_out_off(:,1),'-'); legend('LF ON','LF OFF'); xlabel('Speed (m/s)'); ylabel('Slip angle (deg)');

% Save sim report to workspace
simReport.lap_time = laptime;
simReport.drs_active_for_segment = drs_active_for_segment;
simReport.time_vec = time_vec;
simReport.speed_vec = speed_vec;
assignin('base','simReport',simReport);
fprintf('Sim complete. simReport saved to workspace.\n');
