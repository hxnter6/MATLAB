% Point_Calculation.m
% to be called by LTS_TUI.m
tbest = min(min(laptime));
endurobest = tbest*enduro_laps;
skidbest = min(min(skidpadtime));
accelbest = min(min(acceltime));
fuelbest = min(min(fuelused_adj));

for iii = 1:i_max
    for jjj = 1:j_max
        E(iii,jjj) = (endurobest*fuelbest)/(endurotime(iii,jjj)*fuelused_adj(iii,jjj));
    end
end
Emax = max(E);
Emin = min(E);

for iii = 1:i_max
    for jjj = 1:j_max
        
        
        autox_pts(iii,jjj) = 95.5*((((tbest*1.25)/laptime(iii,jjj))-1)/0.25) + 4.5;
        enduro_pts(iii,jjj) = 300*((((endurobest*1.333)/endurotime(iii,jjj))-1)/0.333) + 25;
        skidpad_pts(iii,jjj) = 71.5*((((skidbest*1.25)/skidpadtime(iii,jjj))^2-1)/0.5625) + 3.5;
        accel_pts(iii,jjj) = 71.5*(((accelbest*1.5/acceltime(iii,jjj))-1)/0.5) + 3.5;
%         eff_pts(iii,jjj) = 100*((Emin/E(iii,jjj)-1)/(Emin/Emax-1));
        
        total_pts(iii,jjj) = autox_pts(iii,jjj)+enduro_pts(iii,jjj)+skidpad_pts(iii,jjj)+accel_pts(iii,jjj);%+eff_pts(iii,jjj);
        
    end
end