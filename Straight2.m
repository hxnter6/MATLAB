function [ time,velocity,throttle] = Straight2( dist,dt,Ax_drive,Ax_brake,v_initial,v_final,vtol)
% Simulates straight for LTS
% Also Calculates "ideal" braking point
% Needs to be cleaned up, but currently works

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
        t_d(j+1) = t_d(j)+dt;
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
    k_last = k;
    x_accb = d(i);
    t_b(k) = t_d(j-1);
    x_brake(k) = sum(d(1:i));
    xd_brake_1(k) = v_final;
    xdd_brake(k) = 0;
    while abs(x_brake(k)) <= d(i)
        t_b(k+1) = t_b(k)-dt;
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
    
    throttle = zeros(1,ii_max);
    velocity = zeros(1,ii_max);
    jj = 1;
    time(1) = 0;
    
    for ii = 1:ii_max-1
        % Braking
        if xd_drive(ii) >= xd_brake_flip(ii) 
            throttle(ii) = 0;
            velocity(ii) = xd_brake_flip(ii);
        % Driving
        elseif xd_drive(ii) < xd_brake_flip(ii)
            throttle(ii) = 1;
            velocity(ii) = xd_drive(ii);
        else
            error('fuck')
        end
        
        if throttle(ii) ~= 1 && throttle(ii) ~= 0
            error('"var" throttle is binary and contains non-binary values')
        end
        time(jj+1) = time(jj) + dt;
        jj = jj+1;
    end
%     time
end