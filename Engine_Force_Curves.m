%% Engine_Force_Curves.m
% To be called by LTS
ploton = 0;
fprintf('Loading Engine Model...')

Engine_Text = fopen(Enginecode);
out = textscan(Engine_Text,'%s');
text = string(out{1,1});
fuelcode = text(5);
values = csvread(Enginecode,3,0);
engine_spd = values(:,1)'; %rpm
power = values(:,2)'; %hp
torque = values(:,3)'; %ft.lbs
fuel = values(:,4)'; %L/hr
gearing = values(:,5)';
gearing = gearing(gearing~=0);
gearnum = numel(gearing)-1;
shifting = values(:,6)';
shifting = shifting(shifting~=0);

tol = engine_spd(1)-engine_spd(2);

% Converting from freedom units and linear scaling
engine_name = strjoin([text(1) text(2) text(3) text(4) text(5) text(6)]);
power = power.* 0.745699872 * power_coeff; %kW
torque = torque.* 1.35582 * power_coeff; %N.m
shifting = shifting.* 0.44704; %m/s
fuel = fuel/3600; %L/s
primary = gearing(1);
curr_gear = 1;

for ii = 1:length(v_range)
    v = v_range(ii);
    if gearnum == 1
        gear_pos(ii) = curr_gear;
        rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*(60/(2*pi))/r_tire),tol);
        ind = find(engine_spd <= rpm(ii));
        Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*finaldrive/r_tire;
    elseif v < shifting(curr_gear)
        gear_pos(ii) = curr_gear;
        rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*(60/(2*pi))/r_tire),tol);
        ind = find(engine_spd <= rpm(ii));
        fuel_flow(ii) = fuel(ind(1));
        Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*finaldrive/r_tire;
    elseif v > shifting(curr_gear) && v < shifting(length(shifting))
        curr_gear = curr_gear + 1;
        gear_pos(ii) = curr_gear;
        rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*(60/(2*pi))/r_tire),tol);
        ind = find(engine_spd <= rpm(ii));
        fuel_flow(ii) = fuel(ind(1));
        Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*finaldrive/r_tire;
    elseif v > shifting(length(shifting))
        curr_gear = curr_gear + 1;
        gear_pos(ii) = curr_gear;
        rpm(ii) = round2((v*primary*gearing(gear_pos(ii)+1)*finaldrive*(60/(2*pi))/r_tire),tol);
        ind = find(engine_spd <= rpm(ii));
        fuel_flow(ii) = fuel(ind(1));
        Fx_engine(ii) = torque(ind(1))*primary*gearing(gear_pos(ii)+1)*finaldrive/r_tire;
        curr_gear = curr_gear - 1;
    end
end

for ii = 1:length(v_range)
    if rpm(ii) > rpm_limit
        Fx_engine(ii) = 0;
    end
end


fprintf(' complete.\n')

if ploton
    
    for curr_gear = 1:gearnum
        % Velocity at each rpm step per gear
        v_(curr_gear,:) = engine_spd./primary./gearing(curr_gear+1)/finaldrive*(2*pi/60)*r_tire;
        
        % Force at tire at each rpm step per gear
        f_(curr_gear,:) = torque.*primary.*gearing(curr_gear+1)*finaldrive/r_tire;
    end
    
    
    figure(1)
    plot(engine_spd,power)
    grid on
    hold on
    title([engine_name])
    xlabel('Engine Speed (RPM)')
    ylabel('Power (KW) Torque(N.m)')
    plot(engine_spd,torque)
    legend('Power (kW)', 'Torque')

    figure(2)
    scatter(v_(1,:),f_(1,:),10,engine_spd)
    hold on
    if numel(gearing) > 2
        scatter(v_(2,:),f_(2,:),10,engine_spd)
    end
    if numel(gearing) > 3
        scatter(v_(3,:),f_(3,:),10,engine_spd)
    end
    if numel(gearing) > 4
        scatter(v_(4,:),f_(4,:),10,engine_spd)
    end
    if numel(gearing) > 5
        scatter(v_(5,:),f_(5,:),10,engine_spd)
    end
    if numel(gearing) > 6
        scatter(v_(6,:),f_(6,:),10,engine_spd)
    end
    grid on
    colormap jet
    c = colorbar;
    title([engine_name ' Force Curve']) 
    xlabel('Speed (m/s)')
    ylabel('Force at Contact Patch (N)')
    ylabel(c,'Engine RPM')
    legend off

%     figure(3);
%     plot(v_range,Fx_engine);
%     grid on
%     title('Combined Force Curves')
%     xlabel('Speed (m/s)')
%     ylabel('Force at Contact Patch (N)')
%     
%     figure(4)
%     plot(v_range,fuel_flow)
%     
%     figure(5)
%     plot(v_range,gear_pos)
end         
