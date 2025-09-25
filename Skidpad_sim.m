%% Skidpad.m
%To be called by LTS_TUI
fprintf('Simulating Skidpad... ')
v_check = 0;
corner_conv = 0;
r_skid = 7.625+track/2;
while corner_conv ~= 1

    [v_skid, time_skid] = corner(Ay(v_check),r_skid,r_skid*2*pi,roundoff,dt,0);
    v_i = mean(v_skid);
    if abs(v_check-v_i) >= v_tol
        v_check = abs(v_check+v_i)/2;
    else
        corner_conv = 1;
    end
end

skidpadtime(iii,jjj) = max(time_skid);
fprintf('complete.\n')