%% Post_Processing.m
% To be called by LTS

fprintf('Post Processing...')
limits = size(time_out);
time_vec = time_out(1,:);
v_vec = v_out(1,:);
throttle_vec = throttle_out(1,:);
qqq = limits(2);
ref = 0;

% clear time_vec v_vec throttle_vec time speed throttle

% Converting time and velocity matrices to vector and adjusting time
for q = 1:limits(1)
    for qq = 1:limits(2)
        qqq = qqq+1;
        time_vec(qqq) = time_out(q,qq);
        if time_vec(qqq) ~= 0
            time_vec(qqq) = time_out(q,qq) + ref;
        end
        v_vec(qqq) = v_out(q,qq);
        throttle_vec(qqq) = throttle_out(q,qq);
    end
    ref = max(time_out(q,:)) + ref;
end

% Isolating nonzero values
zz = 0;
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
speed(1) = speed(2);

% % Fuel Usage
% for x = 1:zz
%     speed_round(x) = round2(speed(x),dv);
%     ref = find(v_range >= speed_round(x));
%     fuel_out(x) = fuel_flow(ref(1))*throttle(x);
% end
% 
% fuel_calc(iii,jjj) = 0;
% 
% for xx = 2:numel(fuel_out)
%     fuel_calc(iii,jjj) = fuel_calc(iii,jjj) + 0.5*dt*(fuel_out(xx-1)+fuel_out(xx)); % Fuel used for a single lap
% end
% 
% fuelused(iii,jjj) = 1.00*fuel_calc(iii,jjj)*enduro_laps;
% 
% % CO2 usage based on fuel type
% if fuelcode == 'E85'
%     fuelused_adj(iii,jjj) = fuelused(iii,jjj)/1.4;
% else % Gasoline
%     fuelused_adj(iii,jjj) = fuelused(iii,jjj);
% end

fuelused_adj(iii,jjj) = 0;

laptime(iii,jjj) = max(time);
endurotime(iii,jjj) = laptime(iii,jjj)*enduro_laps;
fprintf(' complete. \n\n')
