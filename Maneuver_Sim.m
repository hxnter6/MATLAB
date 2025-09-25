%%
% Maneuver_Sim.m
% To be called by LTS.m
fprintf('Simulating Lap...')

% clear time_out v_out throttle_out

for i = 1:numel(d)

    if r(i) > 0 % Cornering case
        v_check = 0;
        corner_conv = 0;
        while corner_conv ~= 1
            
            [v_c, time_c, throttle_c] = corner(Ay(v_check),r(i),d(i),roundoff,dt,0);
            v_i = mean(v_c);
            if abs(v_check-v_i) >= v_tol
                v_check = abs(v_check+v_i)/2;
            else
                corner_conv = 1;
            end
        end
        
        % Generating Lap Reports
        for j = 1:numel(time_c)
            time_out(i,j) = time_c(j);
            if v_c(j) >= v_max
                v_out(i,j) = v_max;
            else
                v_out(i,j) = v_c(j);
            end
            throttle_out(i,j) = throttle_c(j);
            manuver_num(i,j) = i;
        end
            
    elseif r(i) == 0 % Straight case
        % Forcing a rolling start to lap
        
        % Lap Start
        if i == 1 
            % Exit Velocity
            v_check = 0;
            corner_conv = 0;
            while corner_conv ~= 1

                v_last = corner_simple(Ay(v_check),r(numel(d)),d(numel(d)),roundoff,dt,0);
                v_i = mean(v_last);

                if abs(v_check-v_i) >= v_tol
                    v_check = abs(v_check+v_i)/2;
                else
                    corner_conv = 1;
                end
            end
            
            v_exit = mean(v_last);
            
            % Entry Velocity
            v_check = 0;
            corner_conv = 0;
            while corner_conv ~= 1
            
                v_next = corner_simple(Ay(v_check),r(i+1),d(i+1),roundoff,dt,0);
                v_i = mean(v_next);

                if abs(v_check-v_i) >= v_tol
                    v_check = abs(v_check+v_i)/2;
                else
                    corner_conv = 1;
                end
            end
            v_entry = mean(v_next);
            
        % Mid Lap
        elseif i == numel(d)
            
            % Exit Velocity
            v_check = 0;
            corner_conv = 0;
            while corner_conv ~= 1
            
                v_last = corner_simple(Ay(v_check),r(i-1),d(i-1),roundoff,dt,0);
                v_i = mean(v_last);

                if abs(v_check-v_i) >= v_tol
                    v_check = abs(v_check+v_i)/2;
                else
                    corner_conv = 1;
                end
            end

            v_exit  = mean(v_last);
            
            % Entry Velocity
            v_check = 0;
            corner_conv = 0;
            while corner_conv ~= 1
            
                v_next = corner_simple(Ay(v_check),r(1),d(1),roundoff,dt,0);
                v_i = mean(v_next);

                if abs(v_check-v_i) >= v_tol
                    v_check = abs(v_check+v_i)/2;
                else
                    corner_conv = 1;
                end
            end
            v_entry = mean(v_next);
        
        % Lap End
        else
            % Exit Velocity
            v_check = 0;
            corner_conv = 0;
            while corner_conv ~= 1
            
                v_last = corner_simple(Ay(v_check),r(i-1),d(i-1),roundoff,dt,0);
                v_i = mean(v_last);

                if abs(v_check-v_i) >= v_tol
                    v_check = abs(v_check+v_i)/2;
                else
                    corner_conv = 1;
                end
            end
            
            v_exit  = mean(v_last);
            
            % Entry Velocity
            v_check = 0;
            corner_conv = 0;
            while corner_conv ~= 1
            
                v_next = corner_simple(Ay(v_check),r(i+1),d(i+1),roundoff,dt,0);
                v_i = mean(v_next);

                if abs(v_check-v_i) >= v_tol
                    v_check = abs(v_check+v_i)/2;
                else
                    corner_conv = 1;
                end
            end
            v_entry = mean(v_next);
        end
        
        [time_s, v_s, throttle_s] = Straight2(d(i),dt,Ax_drive,Ax_brake,v_exit,v_entry,v_tol);
        
        % Generating Lap Report
        for k = 1:numel(time_s)
            time_out(i,k) = time_s(k);
            v_out(i,k) = v_s(k);
            throttle_out(i,k) = throttle_s(k);
        end
    end
end
fprintf(' complete.\n')
