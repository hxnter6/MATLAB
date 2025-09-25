%% Accel_sim.m
% To be called by LTS_TUI.m
fprintf('Simulating Accel... ')
acceltime(iii,jjj) = 0;
w = 1;
d_accel(1) = 0;
xd_accel(1) = 5; % This is to force the vehicle model to output positive value
xdd_accel(1) = 0;
while d_accel(w) <= 76
    
    acceltime(iii,jjj) = acceltime(iii,jjj) + dt;
    xdd_accel(w+1) = abs(Ax_drive(xd_accel(w)));
    xd_accel(w+1) = xd_accel(w) + 0.5*dt*(xdd_accel(w+1)+xdd_accel(w));
    d_accel(w+1) = d_accel(w) + 0.5*dt*(xd_accel(w+1)+xd_accel(w));
    
    w = w+1;
%     if acceltime(iii,jjj) > 100
%         error('Vehicle model showing negative value')
%     end
end

acceltime(iii,jjj) = acceltime(iii,jjj) + shift_time*numel(shifting);
fprintf('complete.\n')