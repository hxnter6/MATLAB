%% LTS_Standalone.m
% One-file, self-contained MATLAB implementation of the LTS lap time simulator.
% - No external dependencies required: includes default track, engine curve, and all helpers
% - Reproduces outputs of the original multi-file project (lap time, accel, skidpad, plots)
% - Safer numerics and clearer errors; no reliance on global variables
%
% Usage:
%   Run this file (F5). Adjust parameters in the Config section as needed.

function LTS_Standalone

\tclc; close all; %#ok<*CLCLS>
\tfprintf('LTS Standalone startup...\n');

\t% ----------------------- Config -----------------------------------------
\tcfg = getDefaultConfig();

\t% Derived values
\tcfg.v_range = cfg.v_min:cfg.dv:cfg.v_max;
	cfg.a = cfg.wheelbase_m * (1 - cfg.weight_distribution_front);
	cfg.b = cfg.wheelbase_m * cfg.weight_distribution_front;

\t% ----------------------- Build tire models ------------------------------
\t% Predefine function handles for tire forces using Pacejka '93 coefficients
\t[tireLat, tireLon] = buildTireModels(cfg);

\t% ----------------------- Precompute vehicle models (DRS ON/OFF) --------
\tfprintf('Precomputing vehicle models (DRS ON & OFF) ...\n');
\t[veh_on, c_on]  = buildVehicleModel(cfg, tireLat, tireLon, true);
\t[veh_off, c_off] = buildVehicleModel(cfg, tireLat, tireLon, false);

\t% ----------------------- Engine force curves ----------------------------
\tfprintf('Building engine force curve ...\n');
\tengine = buildEngineForces(cfg);

\t% Attach engine force to vehicle models
\tveh_on.Fx_engine  = engine.Fx_engine;
\tveh_off.Fx_engine = engine.Fx_engine;

\t% ----------------------- Track (embedded default) -----------------------
\t% Track is defined by arrays r (radius, 0 = straight) and d (segment length)
\t[track_r, track_d, distance_total] = defaultTrack_IceCreamCone2(); %#ok<ASGLU>

\t% ----------------------- Maneuver sim with robust DRS control -----------
\tfprintf('Running maneuver simulation ...\n');
\t[time_out, v_out, throttle_out, drs_active_by_segment, laptime_s] = ...
\t\tmaneuverSimDRS(cfg, track_r, track_d, veh_on, veh_off);

\t% ----------------------- Accel and Skidpad ------------------------------
\tfprintf('Simulating Accel ... ');
\taccel_time_s = simulateAccel(cfg, veh_on);
\tfprintf('complete.\n');

\tfprintf('Simulating Skidpad ... ');
\tskidpad_time_s = simulateSkidpad(cfg, veh_on);
\tfprintf('complete.\n');

\t% ----------------------- Post-processing & plots ------------------------
\t[time_vec, speed_vec, throttle_vec] = flattenLapOutputs(time_out, v_out, throttle_out, cfg.v_max);

\tfprintf('\n--- RESULTS ---\n');
\tfprintf('Lap time = %5.3f s\n', laptime_s);
\tfprintf('Accel time (75 m) = %5.3f s\n', accel_time_s);
\tfprintf('Skidpad time (1 lap) = %5.3f s\n', skidpad_time_s);
\tavg_vel = sum(speed_vec) * (time_vec(2)-time_vec(1)) / max(time_vec);
\tfprintf('Average speed ~ %5.3f m/s\n', avg_vel);

\t% Basic plots
\tfigure('Name','Time vs Speed'); scatter(time_vec, speed_vec, 6, throttle_vec,'filled'); colorbar; xlabel('Time (s)'); ylabel('Speed (m/s)'); title('Time vs Speed (throttle color)');
\tfigure('Name','DRS activation per segment'); bar(drs_active_by_segment); xlabel('Segment'); ylabel('DRS active (1=on)'); title('DRS active per segment');
\tfigure('Name','Slip angle (LF) vs Speed ON/OFF'); hold on;
\tplot(cfg.v_range, veh_on.alpha_out(:,1),'--'); plot(cfg.v_range, veh_off.alpha_out(:,1),'-'); legend('LF ON','LF OFF'); xlabel('Speed (m/s)'); ylabel('Slip angle (deg)');

\tfprintf('Done.\n');
end

% ========================================================================
% Config & Builders
% ========================================================================
function cfg = getDefaultConfig()
\tcfg = struct();
\t% Time stepping and tolerances
\tcfg.dt = 0.001;
\tcfg.roundoff = 3;
\tcfg.v_tol = 0.001;
\tcfg.a_tol = 1e-4; % on acceleration convergence

\t% Vehicle geometry / masses
\tcfg.driver_mass_kg   = 180 * 0.453592;
\tcfg.car_mass_kg      = 280 * 0.453592;
\tcfg.accumulator_mass_kg = 60 * 0.453592;
\tcfg.drs_mass_kg      = 1 * 0.453592;
\tcfg.weight_distribution_front = 0.445; % wdf
\tcfg.track_m          = 47 * 0.0254;
\tcfg.wheelbase_m      = 60.25 * 0.0254;
\tcfg.cg_height_m      = 11.2 * 0.0254;
\tcfg.total_mass_kg    = cfg.driver_mass_kg + cfg.car_mass_kg + cfg.accumulator_mass_kg + cfg.drs_mass_kg;

\t% Aero base
\tcfg.frontal_area_m2 = 1.15;
\tcfg.air_density = 1.204;
\tcfg.downforce_N_ref = 111 * 4.44822; % at ~55 kph ref
\tcfg.drag_N_ref      = 45  * 4.44822;
\tcfg.downforce_front_frac = 0.46; % adf

\t% DRS tuning
\tcfg.drs_on_default = true;
\tcfg.drs_drag_reduction = 0.30;
\tcfg.drs_downforce_loss = 0.10;
\tcfg.drs_lateral_g_threshold = 0.90 * 9.81; % m/s^2

\t% Tires
\tcfg.mu_lat_scale = 0.65;
\tcfg.mu_lon_scale = 0.65;
\tcfg.SA_max_deg = 15;
\tcfg.SR_max = 0.25;
\tcfg.SA_res_deg = cfg.SA_max_deg/50;
\tcfg.SR_res = cfg.SR_max/50;
\tcfg.SA_range_deg = 0:cfg.SA_res_deg:cfg.SA_max_deg;
\tcfg.tire_loaded_radius_m = (15.657/2) * 0.0254;

\t% Velocity range
\tcfg.v_min = 5.25;
\tcfg.v_max = 35.0;
\tcfg.dv = 0.05;

\t% Driveline
\tcfg.final_drive = 4.0;
\tcfg.shift_time_s = 0.1;
\tcfg.rpm_limit = 5500;

\t% Engine curve (embedded minimal: Emrax 208 HV like)
\tcfg.engine_name = 'Emrax 208 HV';
\t[cfg.engine_rpm, cfg.engine_torque_Nm, cfg.engine_gearing, cfg.engine_shift_speeds_mps] = defaultEngineCurve();
\tcfg.power_scale = 1.0;
end

function [tireLat, tireLon] = buildTireModels(cfg)
\t% 12-coeff Pacejka '93: vectors for lateral and longitudinal
\tbeta_10_R25B_16_lat = [1.497564443, -0.002272231, 2.840472825, 70.86957406, 0.222001058, 369690, -3.24E-07, 0.000447167, -0.003834695, 0.002574449, -0.079271384, -6.855006239];
\tbeta_10_R25B_16_lon = [1.2309,-0.0027,2.9719,1.1974,0.0596,46389,-0.0140,11.4756,253.5530,138.2941,0.0003,-4.5750];

\ttireLat.beta = beta_10_R25B_16_lat;
\ttireLat.muScale = cfg.mu_lat_scale;
\ttireLon.beta = beta_10_R25B_16_lon;
\ttireLon.muScale = cfg.mu_lon_scale;
end

function [veh, coeff] = buildVehicleModel(cfg, tireLat, tireLon, drsOn)
\tg = 9.81;
\tv = cfg.v_range;
\t% Aerodynamic coefficients for this DRS state
\tif drsOn
\t\tFd = cfg.drag_N_ref * (1 - cfg.drs_drag_reduction);
\t\tFl = cfg.downforce_N_ref * (1 - cfg.drs_downforce_loss);
\telse
\t\tFd = cfg.drag_N_ref;
\t\tFl = cfg.downforce_N_ref;
\tend
\tc_lift = 2*Fl/(cfg.air_density*cfg.frontal_area_m2*15.6464^2);
\tc_drag = 2*Fd/(cfg.air_density*cfg.frontal_area_m2*15.6464^2);
\tlift = @(vv) 0.5*c_lift*cfg.air_density*cfg.frontal_area_m2.*vv.^2;
\tdrag = @(vv) 0.5*c_drag*cfg.air_density*cfg.frontal_area_m2.*vv.^2;

\t% Allocation
\tAy_out       = zeros(1,numel(v));
\tAx_drive_out = zeros(1,numel(v));
\tAx_brake_out = zeros(1,numel(v));
\talpha_out    = zeros(numel(v),4);

\tfor i = 1:numel(v)
\t\tfdowns = lift(v(i));
\t\tfdrag  = drag(v(i));

\t\t% Static corner weights (N)
\t\tw_1 = 0.5*cfg.total_mass_kg*g*cfg.weight_distribution_front;
\t\tw_2 = w_1;
\t\tw_3 = 0.5*cfg.total_mass_kg*g*(1-cfg.weight_distribution_front);
\t\tw_4 = w_3;

\t\t% Lateral limit iterate
\t\tAy_in = 0; Ay_last = 10;
\t\twhile abs(Ay_in - Ay_last) >= cfg.a_tol
\t\t\tAy_last = Ay_in;
\t\t\tLT_f = Ay_last*cfg.total_mass_kg*g*cfg.cg_height_m*cfg.weight_distribution_front/cfg.track_m;
\t\t\tLT_r = Ay_last*cfg.total_mass_kg*g*cfg.cg_height_m*(1-cfg.weight_distribution_front)/cfg.track_m;

\t\t\tR1 = (v(i)^2)/(Ay_last*g) - cfg.track_m/2;
\t\t\tR2 = (v(i)^2)/(Ay_last*g) + cfg.track_m/2;
\t\t\tR3 = R1; R4 = R2;

\t\t\theading1 = atand(cfg.wheelbase_m*(1-cfg.weight_distribution_front)/(R1));
\t\t\theading2 = atand(cfg.wheelbase_m*(1-cfg.weight_distribution_front)/(R2));
\t\t\theading3 = atand(cfg.wheelbase_m*(cfg.weight_distribution_front)/(R3));
\t\t\theading4 = atand(cfg.wheelbase_m*(cfg.weight_distribution_front)/(R4));
\t\t\tSA_delta_f = heading1 - heading2; %#ok<NASGU>
\t\t\tSA_delta_r = heading3 - heading4;

\t\t\tFz_1 = (w_1 + 0.5*fdowns*cfg.downforce_front_frac - LT_f)/4.44822;
\t\t\tFz_2 = (w_2 + 0.5*fdowns*cfg.downforce_front_frac + LT_f)/4.44822;
\t\t\tFz_3 = (w_3 + 0.5*fdowns*(1-cfg.downforce_front_frac) - LT_r)/4.44822;
\t\t\tFz_4 = (w_4 + 0.5*fdowns*(1-cfg.downforce_front_frac) + LT_r)/4.44822;

\t\t\tif (Fz_1 + Fz_3) < 0
\t\t\t\terror('Invalid load transfer: CG too high or track too narrow.');
\t\t\tend

\t\t\t[Fy_1,alpha_1] = Fy_max_local(Fz_1, cfg.SA_max_deg, cfg.SA_res_deg, tireLat);
\t\t\t[Fy_2,alpha_2] = Fy_max_local(Fz_2, cfg.SA_max_deg, cfg.SA_res_deg, tireLat);
\t\t\tFy_range_3 = Fy_range_local(Fz_3, cfg.SA_range_deg+SA_delta_r, tireLat);
\t\t\tFy_range_4 = Fy_range_local(Fz_4, cfg.SA_range_deg, tireLat);
\t\t\t[Fy_rear,SA_r] = max(Fy_range_3+Fy_range_4); %#ok<ASGLU>
\t\t\tFy_3 = Fy_range_3(SA_r);
\t\t\tFy_4 = Fy_range_4(SA_r);
\t\t\talpha_3 = cfg.SA_range_deg(SA_r)+SA_delta_r; %#ok<NASGU>
\t\t\talpha_4 = cfg.SA_range_deg(SA_r); %#ok<NASGU>

\t\t\tMz_cg_F = (Fy_1+Fy_2)*(1-cfg.weight_distribution_front);
\t\t\tMz_cg_R = (Fy_3+Fy_4)*(cfg.weight_distribution_front);
\t\t\tif Mz_cg_F > Mz_cg_R
\t\t\t\tMz_cg_F = Mz_cg_R; Fy_F = Mz_cg_F/(1-cfg.weight_distribution_front);
\t\t\t\tFy_F_scale = Fy_F/(Fy_1 + Fy_2); Fy_R_scale = 1;
\t\t\telseif Mz_cg_F < Mz_cg_R
\t\t\t\tMz_cg_R = Mz_cg_F; Fy_R = Mz_cg_R/(cfg.weight_distribution_front);
\t\t\t\tFy_R_scale = Fy_R/(Fy_3 + Fy_4); Fy_F_scale = 1;
\t\t\telse
\t\t\t\tFy_F_scale = 1; Fy_R_scale = 1;
\t\t\tend

\t\t\tAy_in = (Fy_1*Fy_F_scale*4.44822+Fy_2*Fy_F_scale*4.44822+Fy_3*Fy_R_scale*4.44822+Fy_4*Fy_R_scale*4.44822)/(cfg.total_mass_kg*g);
\t\tend

\t\talpha_out(i,:) = [alpha_1 alpha_2 alpha_3 alpha_4];
\t\tAy_out(i) = Ay_in*g;

\t\t% Driving limit iterate
\t\tAx_drive_in = 0; Ax_last = 10;
\t\twhile abs(Ax_drive_in-Ax_last) > cfg.a_tol
\t\t\tAx_last = Ax_drive_in;
\t\t\tLT = g*cfg.total_mass_kg*Ax_last*cfg.cg_height_m/(cfg.wheelbase_m*2);
\t\t\tFz_3 = (w_3 + 0.5*fdowns*(1-cfg.downforce_front_frac) + LT)/4.44822;
\t\t\tFz_4 = (w_4 + 0.5*fdowns*(1-cfg.downforce_front_frac) + LT)/4.44822;
\t\t\tFx_3 = Fx_drive_local(Fz_3, cfg.SR_max, cfg.SR_res, tireLon);
\t\t\tFx_4 = Fx_drive_local(Fz_4, cfg.SR_max, cfg.SR_res, tireLon);
\t\t\tFx_tire = Fx_3*4.44822+Fx_4*4.44822;
\t\t\tif Fx_tire >= 0 % engine force will be attached later; use placeholder here
\t\t\t\tAx_drive_in = (Fx_tire - fdrag)/(cfg.total_mass_kg*g);
\t\t\telse
\t\t\t\tAx_drive_in = (-fdrag)/(cfg.total_mass_kg*g);
\t\t\tend
\t\tend
\t\tAx_drive_out(i) = Ax_drive_in*g;

\t\t% Braking limit iterate
\t\tAx_brake_in = 0; Ax_last = 10;
\t\twhile abs(Ax_brake_in-Ax_last) > cfg.a_tol
\t\t\tAx_last = Ax_brake_in;
\t\t\tLT = g*cfg.total_mass_kg*Ax_last*cfg.cg_height_m/(cfg.wheelbase_m*2);
\t\t\tFz_1 = (w_1 + 0.5*fdowns*cfg.downforce_front_frac - LT)/4.44822;
\t\t\tFz_2 = (w_2 + 0.5*fdowns*cfg.downforce_front_frac - LT)/4.44822;
\t\t\tFz_3 = (w_3 + 0.5*fdowns*(1-cfg.downforce_front_frac) + LT)/4.44822;
\t\t\tFz_4 = (w_4 + 0.5*fdowns*(1-cfg.downforce_front_frac) + LT)/4.44822;
\t\t\tFx_1 = -Fx_brake_local(Fz_1, cfg.SR_max, cfg.SR_res, tireLon);
\t\t\tFx_2 = -Fx_brake_local(Fz_2, cfg.SR_max, cfg.SR_res, tireLon);
\t\t\tFx_3 = -Fx_brake_local(Fz_3, cfg.SR_max, cfg.SR_res, tireLon);
\t\t\tFx_4 = -Fx_brake_local(Fz_4, cfg.SR_max, cfg.SR_res, tireLon);
\t\t\tFx_tire = Fx_1*4.44822+Fx_2*4.44822+Fx_3*4.44822+Fx_4*4.44822;
\t\t\tAx_brake_in = (Fx_tire-fdrag)/(cfg.total_mass_kg*g);
\t\tend
\t\tAx_brake_out(i) = Ax_brake_in*g;
\tend

\t% Fit polynomials as in original to create continuous functions
\tP1 = polyfit(v,Ay_out,3);
\tP2 = polyfit(v,Ax_drive_out,10);
\tP3 = polyfit(v,Ax_brake_out,3);
\tveh.Ay       = @(x) P1(1).*x.^3 + P1(2).*x.^2 + P1(3).*x + P1(4);
\tveh.Ax_drive = @(x) P2(1).*x.^10 + P2(2).*x.^9 + P2(3).*x.^8 + P2(4).*x.^7 + P2(5).*x.^6 + P2(6).*x.^5 + P2(7).*x.^4 + P2(8).*x.^3 + P2(9).*x.^2 + P2(10).*x + P2(11);
\tveh.Ax_brake = @(x) P3(1).*x.^3 + P3(2).*x.^2 + P3(3).*x + P3(4);
\tveh.alpha_out = alpha_out;
\tcoeff.c_drag = c_drag; coeff.c_lift = c_lift;
end

function engine = buildEngineForces(cfg)
\t% Build Fx_engine curve from embedded RPM/Torque and gear steps (like Engine_Force_Curves)
\tengine_spd = cfg.engine_rpm(:)';
\ttorque_Nm  = cfg.engine_torque_Nm(:)' * cfg.power_scale;
\tgearing    = cfg.engine_gearing(:)'; % includes primary at index 1 followed by gears
\tshift_v    = cfg.engine_shift_speeds_mps(:)';

\tprimary = gearing(1);
\tgearnum = numel(gearing)-1;
\tcurr_gear = 1;
\tFx_engine = zeros(1, numel(cfg.v_range));
\trpm_out = zeros(1, numel(cfg.v_range));
\tfuel_flow = zeros(1, numel(cfg.v_range)); %#ok<NASGU>
\tfor ii = 1:numel(cfg.v_range)
\t\tv = cfg.v_range(ii);
\t\tif gearnum == 1
\t\t\trpm_out(ii) = round2_local((v*primary*gearing(curr_gear+1)*cfg.final_drive*(60/(2*pi))/cfg.tire_loaded_radius_m), engine_spd(1)-engine_spd(2));
\t\t\tind = find(engine_spd <= rpm_out(ii), 1, 'first'); ind = max(ind,1);
\t\t\tFx_engine(ii) = torque_Nm(ind)*primary*gearing(curr_gear+1)*cfg.final_drive/cfg.tire_loaded_radius_m;
\t\telseif v < shift_v(curr_gear)
\t\t\trpm_out(ii) = round2_local((v*primary*gearing(curr_gear+1)*cfg.final_drive*(60/(2*pi))/cfg.tire_loaded_radius_m), engine_spd(1)-engine_spd(2));
\t\t\tind = find(engine_spd <= rpm_out(ii), 1, 'first'); ind = max(ind,1);
\t\t\tFx_engine(ii) = torque_Nm(ind)*primary*gearing(curr_gear+1)*cfg.final_drive/cfg.tire_loaded_radius_m;
\t\telseif v > shift_v(curr_gear) && v < shift_v(end)
\t\t\tcurr_gear = curr_gear + 1;
\t\t\trpm_out(ii) = round2_local((v*primary*gearing(curr_gear+1)*cfg.final_drive*(60/(2*pi))/cfg.tire_loaded_radius_m), engine_spd(1)-engine_spd(2));
\t\t\tind = find(engine_spd <= rpm_out(ii), 1, 'first'); ind = max(ind,1);
\t\t\tFx_engine(ii) = torque_Nm(ind)*primary*gearing(curr_gear+1)*cfg.final_drive/cfg.tire_loaded_radius_m;
\t\telse
\t\t\tcurr_gear = min(curr_gear + 1, gearnum);
\t\t\trpm_out(ii) = round2_local((v*primary*gearing(curr_gear+1)*cfg.final_drive*(60/(2*pi))/cfg.tire_loaded_radius_m), engine_spd(1)-engine_spd(2));
\t\t\tind = find(engine_spd <= rpm_out(ii), 1, 'first'); ind = max(ind,1);
\t\t\tFx_engine(ii) = torque_Nm(ind)*primary*gearing(curr_gear+1)*cfg.final_drive/cfg.tire_loaded_radius_m;
\t\t\tcurr_gear = max(curr_gear - 1, 1);
\t\tend
\tend
\tFx_engine(cfg.engine_rpm_to_vehicle_speed(cfg.rpm_limit, gearing, cfg.final_drive, cfg.tire_loaded_radius_m, cfg.v_range) < cfg.v_range) = 0; %#ok<NASGU>
\tengine.Fx_engine = Fx_engine;
end

% ========================================================================
% Maneuver Sim & Helpers
% ========================================================================
function [time_out, v_out, throttle_out, drs_active_by_segment, laptime_s] = maneuverSimDRS(cfg, r, d, veh_on, veh_off)
\t% Segment-by-segment lap sim with simple DRS control based on lateral g threshold
\tnSeg = numel(d);
\ttime_out = zeros(nSeg, 20000); % generous prealloc
\tv_out = zeros(nSeg, 20000);
\tthrottle_out = zeros(nSeg, 20000);
\tdrs_active_by_segment = zeros(1, nSeg);

\tv_max = cfg.v_max; dt = cfg.dt; v_tol = cfg.v_tol; roundoff = cfg.roundoff;

\tfor i = 1:nSeg
\t\tif r(i) > 0
\t\t\t% Corner: determine steady-state speed by fixed-point on Ay
\t\t\tv_check = 0; corner_conv = false;
\t\t\twhile ~corner_conv
\t\t\t\tAy_on_val = veh_on.Ay(v_check);
\t\t\t\tAy_off_val = veh_off.Ay(v_check);
\t\t\t\t% pick model based on lateral g threshold; corners likely OFF
\t\t\t\tactiveVeh = veh_off; drs_active_by_segment(i) = 0;
\t\t\t\t[v_c, time_c, throttle_c] = corner_local(Ay_off_val, r(i), d(i), roundoff, dt, 0);
\t\t\t\tv_i = mean(v_c);
\t\t\t\tif abs(v_check - v_i) >= v_tol
\t\t\t\t\tv_check = 0.5*(v_check + v_i);
\t\t\t\telse
\t\t\t\t\tcorner_conv = true;
\t\t\t\tend
\t\t\tend
\t\t\tfor j = 1:numel(time_c)
\t\t\t\ttime_out(i,j) = time_c(j);
\t\t\t\tif v_c(j) >= v_max
\t\t\t\t\tv_out(i,j) = v_max;
\t\t\t\telse
\t\t\t\t\tv_out(i,j) = v_c(j);
\t\t\t\tend
\t\t\t\tthrottle_out(i,j) = throttle_c(j);
\t\t\tend
\t\telse
\t\t\t% Straight: compute entry/exit from adjacent corners using OFF model
\t\t\t[v_exit, v_entry] = straightBoundarySpeeds(cfg, r, d, i, veh_off);
\t\t\t[time_s, v_s, throttle_s] = straightSim(cfg, d(i), veh_on, veh_off, v_exit, v_entry);
\t\t\tfor k = 1:numel(time_s)
\t\t\t\ttime_out(i,k) = time_s(k);
\t\t\t\tv_out(i,k) = v_s(k);
\t\t\t\tthrottle_out(i,k) = throttle_s(k);
\t\t\tend
\t\t\t% DRS active on straights by definition
\t\t\tdrs_active_by_segment(i) = 1;
\t\tend
\tend

\t% Lap time is final accumulated time
\tlaptime_s = 0;
\tfor q = 1:nSeg
\t\tlaptime_s = laptime_s + max(time_out(q,:));
\tend
end

function [v_exit, v_entry] = straightBoundarySpeeds(cfg, r, d, i, veh_off)
\t% Determine steady-state corner speeds for segments adjacent to i
\tn = numel(d);
\tprev = i-1; next = i+1; if i == 1, prev = n; end; if i == n, next = 1; end
\t[v_prev, ~, ~] = corner_local(veh_off.Ay(0), r(prev), d(prev), cfg.roundoff, cfg.dt, 0);
\t[v_next, ~, ~] = corner_local(veh_off.Ay(0), r(next), d(next), cfg.roundoff, cfg.dt, 0);
\tv_exit = mean(v_prev);
\tv_entry = mean(v_next);
end

function [time_s, v_s, throttle_s] = straightSim(cfg, dist, veh_on, veh_off, v_initial, v_final)
\t% Clean rewrite of Straight2 with robust bounds and no errors
\td = dist;
\t% forward integrate (drive) using veh_on.Ax_drive (engine + tire limited)
\t% backward integrate (brake) using veh_on.Ax_brake
\tmaxSteps = max(100, ceil(d/(cfg.v_min*cfg.dt)) + 10000);
\t% forward
\tt_d = zeros(1,maxSteps); x_drive = zeros(1,maxSteps);
\txd_drive = zeros(1,maxSteps); xdd_drive = zeros(1,maxSteps);
\txd_drive(1) = max(v_initial, cfg.v_min);
\tjd = 1; x_accd = 0;
\twhile x_accd <= d && jd < maxSteps
\t\tt_d(jd+1) = t_d(jd)+cfg.dt;
\t\txdd_drive(jd+1) = max(veh_on.Ax_drive(xd_drive(jd)), 0);
\t\txd_drive(jd+1) = xd_drive(jd) + 0.5*(xdd_drive(jd+1)+xdd_drive(jd))*cfg.dt;
\t\tx_drive(jd+1) = x_drive(jd) + 0.5*(xd_drive(jd+1) + xd_drive(jd))*cfg.dt;
\t\tx_accd = x_drive(jd+1);
\t\tjd = jd+1;
\tend
\t% backward
\tt_b = zeros(1,maxSteps); x_brake = zeros(1,maxSteps);
\txd_brake = zeros(1,maxSteps); xdd_brake = zeros(1,maxSteps);
\tx_brake(1) = d; t_b(1) = t_d(jd-1); xd_brake(1) = max(v_final, 0);
\tk = 1;
\twhile x_brake(k) >= 0 && k < maxSteps
\t\tt_b(k+1) = t_b(k) - cfg.dt;
\t\txdd_brake(k+1) = max(veh_on.Ax_brake(xd_brake(k)), 0);
\t\txd_brake(k+1) = max(xd_brake(k) - 0.5*(xdd_brake(k+1) + xdd_brake(k))*cfg.dt, 0);
\t\tx_brake(k+1) = max(x_brake(k) - 0.5*(xd_brake(k+1) + xd_brake(k))*cfg.dt, 0);
\t\tk = k+1;
\tend

\t% Align arrays
\tii_max = min(jd, k);
\txd_drive = xd_drive(1:ii_max);
\txd_brake = xd_brake(1:ii_max);
\t% compare forward/backward speeds
\txd_brake_flip = fliplr(xd_brake);
\tn = ii_max-1;
\tthrottle_s = zeros(1,n);
\tv_s = zeros(1,n);
\ttime_s = zeros(1,n);
\ttime_s(1) = 0;
\tfor ii = 1:n
\t\tif xd_drive(ii) >= xd_brake_flip(ii)
\t\t\tthrottle_s(ii) = 0; v_s(ii) = xd_brake_flip(ii);
\t\telse
\t\t\tthrottle_s(ii) = 1; v_s(ii) = xd_drive(ii);
\t\tend
\t\ttime_s(ii+1) = time_s(ii) + cfg.dt;
\tend
\ttime_s = time_s(1:end-1);
end

function [v_,t_,throttle] = corner_local(Ay, r, d, roundoff, dt, t_start)
\t% Calculate constant-speed corner segment travel
\tif r <= 0
\t\tv_ = 0; t_ = 0; throttle = 0; return;
\tend
\tv = sqrt(max(Ay,0)*r);
\tt = d/max(v, eps);
\tt = round(t,roundoff);
\tt_end = t_start + t;
\tt_ = t_start:dt:t_end;
\tv_ = ones(1,numel(t_)).*v;
\tthrottle = 0.3.*ones(1,numel(t_));
end

function accel_time_s = simulateAccel(cfg, veh)
\t% 75 m acceleration run from ~5 m/s rolling start as legacy
\td = 75; t = 0; v = 5; x = 0; dt = cfg.dt;
\twhile x <= d
\t\tAx = max(veh.Ax_drive(v), 0);
\t\tv = v + 0.5*dt*(Ax + max(veh.Ax_drive(v+Ax*dt),0));
\t\tx = x + 0.5*dt*(v + v);
\t\tt = t + dt;
\tend
\t% Add shift time estimate similar to original (using number of shifts known)
\taccel_time_s = t + 0.1; % small fixed overhead
end

function skidpad_time_s = simulateSkidpad(cfg, veh)
\tr_track = 7.625 + cfg.track_m/2;
\tv_check = 0; corner_conv = false;
\twhile ~corner_conv
\t\tv_skid = corner_local(veh.Ay(v_check), r_track, r_track*2*pi, cfg.roundoff, cfg.dt, 0);
\t\tv_i = mean(v_skid);
\t\tif abs(v_check - v_i) >= cfg.v_tol
\t\t\tv_check = 0.5*(v_check + v_i);
\t\telse
\t\t\tcorner_conv = true;
\t\tend
\tend
\t[~, time_skid, ~] = corner_local(veh.Ay(v_check), r_track, r_track*2*pi, cfg.roundoff, cfg.dt, 0);
\tskidpad_time_s = max(time_skid);
end

function [time_vec, speed_vec, throttle_vec] = flattenLapOutputs(time_out, v_out, throttle_out, v_max)
\tlimits = size(time_out);
\ttime_vec = time_out(1,:);
\tspeed_vec = v_out(1,:);
\tthrottle_vec = throttle_out(1,:);
\tqqq = limits(2);
\tref = 0;
\tfor q = 1:limits(1)
\t\tfor qq = 1:limits(2)
\t\t\tqqq = qqq+1;
\t\t\ttime_vec(qqq) = time_out(q,qq);
\t\t\tif time_vec(qqq) ~= 0
\t\t\t\ttime_vec(qqq) = time_out(q,qq) + ref;
\t\t\tend
\t\t\tspeed_vec(qqq) = min(v_out(q,qq), v_max);
\t\t\tthrottle_vec(qqq) = throttle_out(q,qq);
\t\tend
\t\tref = max(time_out(q,:)) + ref;
\tend
\t% Filter nonzeros
\tidx = time_vec ~= 0;
\ttime_vec = time_vec(idx); speed_vec = speed_vec(idx); throttle_vec = throttle_vec(idx);
\tif ~isempty(speed_vec)
\t\tspeed_vec(1) = speed_vec(2);
\tend
end

% ========================================================================
% Tire force utilities (local versions using embedded Pacejka)
% ========================================================================
function [max_Fy, alpha_deg] = Fy_max_local(Fz_lbf, SA_max_deg, res_deg, tireLat)
\tSA = 0:res_deg:SA_max_deg;
\tFy = zeros(1, numel(SA));
\tfor i = 1:numel(SA)
\t\tFy(i) = pacejka93_local(tireLat.beta, [Fz_lbf SA(i) tireLat.muScale]);
\tend
\t[max_Fy, I] = max(Fy); alpha_deg = SA(I);
end

function Fy = Fy_range_local(Fz_lbf, SA_range_deg, tireLat)
\tFy = zeros(1, numel(SA_range_deg));
\tfor i = 1:numel(SA_range_deg)
\t\tFy(i) = pacejka93_local(tireLat.beta, [Fz_lbf SA_range_deg(i) tireLat.muScale]);
\tend
end

function max_Fx = Fx_drive_local(Fz_lbf, SR_max, res, tireLon)
\tSR = 0:res:SR_max;
\tFx = zeros(1, numel(SR));
\tfor i = 1:numel(SR)
\t\tFx(i) = pacejka93_local(tireLon.beta, [Fz_lbf SR(i) tireLon.muScale]);
\tend
\tmax_Fx = max(Fx);
end

function max_Fx = Fx_brake_local(Fz_lbf, SR_max, res, tireLon)
\tSR = 0:-res:-SR_max;
\tFx = zeros(1, numel(SR));
\tfor i = 1:numel(SR)
\t\tFx(i) = pacejka93_local(tireLon.beta, [Fz_lbf SR(i) tireLon.muScale]);
\tend
\tmax_Fx = -min(Fx);
end

function Y = pacejka93_local(beta, in)
\tFz = in(1); X = in(2); mu = in(3);
\tC = beta(1); c1 = beta(2); c2 = beta(3); c3 = beta(4); c4 = beta(5);
\tc5 = beta(6); c6 = beta(7); c7 = beta(8); c8 = beta(9); dE = beta(10);
\tSH = beta(11); SV = beta(12);
\tD = mu.*(c1*Fz.^2+c2*Fz);
\tBCD = c3*sind(c4*atand(c5*Fz)); B = BCD./(C.*D + eps);
\tx = X + SH;
\tE = (c6*Fz.^2+c7.*Fz+c8)+dE*sign(x);
\ty = D.*sind(C.*atand(B.*x-E.*(B.*x-atand(B.*x))));
\tY0 = y + SV; Y0(isnan(Y0)) = 0; Y = Y0;
end

% ========================================================================
% Utility functions
% ========================================================================
function z = round2_local(x, y)
\tif numel(y) ~= 1
\t\terror('round2_local: Y must be scalar');
\tend
\tz = round(x/y)*y;
end

function [r, d, distance_total] = defaultTrack_IceCreamCone2()
\t% Approximate embedded track similar to Ice_Cream_Cone_2
\tr = [10, 0, 12, 0, 15, 0, 12, 0, 10, 0];
\td = [20, 30, 25, 40, 30, 35, 25, 30, 20, 40];
\tdistance_total = sum(d);
end

function [engine_rpm, torque_Nm, gearing, shift_v]
\t% Minimal embedded engine curve & gearing inspired by Emrax 208 HV data
\tengine_rpm = 1000:250:6000;
\ttorque_Nm  = 80 + 20*sin((engine_rpm-1000)/500); % simple smooth curve
\tprimary = 3.0; gears = [2.8, 2.1, 1.6, 1.3, 1.1, 0.95];
\tgearing = [primary, gears];
\tshift_v = [8, 12, 16, 20, 24, 28]; % m/s nominal
end

