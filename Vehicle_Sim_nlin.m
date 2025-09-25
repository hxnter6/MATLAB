%%
% vehicle_Sim_nlin.m
% SCoats
% To be called by LTS.m

ploton = 0;

g = 9.81; % m/s2
fprintf('Generating Vehicle Model...')

emptygrid    = zeros(1,numel(v_range));
Ay_out       = emptygrid;
Ax_drive_out = emptygrid;
Ax_brake_out = emptygrid;
    
for i = 1:numel(v_range)

    fdowns = lift(v_range(i)); % Downforce (N)
    fdrag  = drag(v_range(i)); % Drag (N)
    
    Ay_in = 0;
    Ay_last = 10;
    Ax_drive_in = 0;
    Ax_brake_in = 0;
    Ax_last = 10;

    % Static Corner Weights (N)
    w_1 = 0.5*m_total*g*wdf;
    w_2 = 0.5*m_total*g*wdf;
    w_3 = 0.5*m_total*g*(1-wdf);
    w_4 = 0.5*m_total*g*(1-wdf);

    % Lateral Limit **** Need to change to MMM ****
    while abs(Ay_in - Ay_last) >= a_tol
        Ay_last = Ay_in;

        LT_f = Ay_last*m_total*g*cg_h*wdf/track;
        LT_r = Ay_last*m_total*g*cg_h*(1-wdf)/track;
        
        R1 = (v_range(i)^2)/(Ay_last*g) - track/2;
        R2 = (v_range(i)^2)/(Ay_last*g) + track/2;
        R3 = (v_range(i)^2)/(Ay_last*g) - track/2;
        R4 = (v_range(i)^2)/(Ay_last*g) + track/2;
        
        heading1 = atand(l*(1-wdf)/(R1));
        heading2 = atand(l*(1-wdf)/(R2));
        heading3 = atand(l*(wdf)/(R3));
        heading4 = atand(l*(wdf)/(R4));
        
        SA_delta_f = heading1 - heading2;
        SA_delta_r = heading3 - heading4;

%       LT_f = Ay_last*m_total*g*cg_h*wdf*rdf/track;
%       LT_r = Ay_last*m_total*g*cg_h*(1-wdf)*(1-rdf)/track;

        Fz_1 = (w_1 + 0.5*fdowns*adf - LT_f)/4.44822;
        Fz_2 = (w_2 + 0.5*fdowns*adf + LT_f)/4.44822;
        Fz_3 = (w_3 + 0.5*fdowns*(1-adf) - LT_r)/4.44822;
        Fz_4 = (w_4 + 0.5*fdowns*(1-adf) + LT_r)/4.44822;
        
        if (Fz_1 + Fz_3) < 0
            error('Either CG is too high or Track is too narrow')
        end
        
        if Fz_1 < 0
            Fz_2 = Fz_2 - Fz_1;
            Fz_1 = 0;
            disp('Fz_1 < 0')
        elseif Fz_2 < 0
            error('Fz_2 < 0')        
        elseif Fz_3 < 0
            Fz_4 = Fz_4 - Fz_3;
            Fz_3 = 0;
            disp('Fz_3 < 0')
        elseif Fz_4 < 0
            error('Fz_4 < 0')
        end
        
        [Fy_1,alpha_1] = Fy_max(Fz_1,SA_max,SA_res);
        [Fy_2,alpha_2] = Fy_max(Fz_2,SA_max,SA_res);
        Fy_range_3 = Fy_range(Fz_3,SA_range+SA_delta_r);
        Fy_range_4 = Fy_range(Fz_4,SA_range);
        [Fy_rear,SA_r] = max(Fy_range_3+Fy_range_4);
        Fy_3 = Fy_range_3(SA_r);
        Fy_4 = Fy_range_4(SA_r);
        alpha_3 = SA_range(SA_r)+SA_delta_r;
        alpha_4 = SA_range(SA_r);
        
        
        %                           >>
        Mz_cg_F = (Fy_1+Fy_2)*(1-wdf);%*cos(atan(((1-wdf)*l)/((v_range(i)^2)/Ay_last)));
        Mz_cg_R = (Fy_3+Fy_4)*(wdf);%*cos(atan(((wdf)*l)/((v_range(i)^2)/Ay_last)));
        
        if Mz_cg_F > Mz_cg_R
            Mz_cg_F = Mz_cg_R;
            Fy_F = Mz_cg_F/(1-wdf);
            Fy_F_scale = Fy_F/(Fy_1 + Fy_2);
            Fy_R_scale = 1;
        elseif Mz_cg_F < Mz_cg_R
            Mz_cg_R = Mz_cg_F;
            Fy_R = Mz_cg_R/(wdf);
            Fy_R_scale = Fy_R/(Fy_3 + Fy_4);
            Fy_F_scale = 1;
        elseif Mz_cg_F == Mz_cg_R
            Fy_F_scale = 1;
            Fy_R_scale = 1;
        end
        
        Ay_in = (Fy_1*Fy_F_scale*4.44822+Fy_2*Fy_F_scale*4.44822+Fy_3*Fy_R_scale*4.44822+Fy_4*Fy_R_scale*4.44822)/(m_total*g);

    end
    alpha_out(i,:) = [alpha_1 alpha_2 alpha_3 alpha_4];
    Ay_out(i) = Ay_in*g;

    % Driving Limit
    while abs(Ax_drive_in-Ax_last) > a_tol
            
        Ax_last = Ax_drive_in;

        LT = g*m_total*Ax_last*cg_h/(l*2);

        Fz_1 = (w_1 + 0.5*fdowns*adf - LT)/4.44822;
        Fz_2 = (w_2 + 0.5*fdowns*adf - LT)/4.44822;
        Fz_3 = (w_3 + 0.5*fdowns*(1-adf) + LT)/4.44822;
        Fz_4 = (w_4 + 0.5*fdowns*(1-adf) + LT)/4.44822;
        
        if (Fz_1 + Fz_2) < 0
            error('Either CG is too far rearward or wheelbase is too short')
        end

        Fx_3 = Fx_drive(Fz_3,SR_max,SR_res);
        Fx_4 = Fx_drive(Fz_4,SR_max,SR_res);
        
        Fx_tire = Fx_3*4.44822+Fx_4*4.44822;
        
        if Fx_tire >= Fx_engine(i)
            Ax_drive_in = (Fx_engine(i) - fdrag)/(m_total*g);
        elseif Fx_tire <= Fx_engine(i)
            Ax_drive_in = (Fx_tire - fdrag)/(m_total*g);
        end
        
    end
    
    Ax_drive_out(i) = Ax_drive_in*g;

    Ax_last = 10;

    % Braking Limit
    while abs(Ax_brake_in-Ax_last) > a_tol

        Ax_last = Ax_brake_in;

        LT = g*m_total*Ax_last*cg_h/(l*2);

        Fz_1 = (w_1 + 0.5*fdowns*adf - LT)/4.44822;
        Fz_2 = (w_2 + 0.5*fdowns*adf - LT)/4.44822;
        Fz_3 = (w_3 + 0.5*fdowns*(1-adf) + LT)/4.44822;
        Fz_4 = (w_4 + 0.5*fdowns*(1-adf) + LT)/4.44822;
        
        if (Fz_3 + Fz_4) < 0
            error('Either CG is too far forward or wheelbase is too short')
        end

        Fx_1 = -Fx_brake(Fz_1,SR_max,SR_res);
        Fx_2 = -Fx_brake(Fz_2,SR_max,SR_res);
        Fx_3 = -Fx_brake(Fz_3,SR_max,SR_res);
        Fx_4 = -Fx_brake(Fz_4,SR_max,SR_res);
        
        Fx_tire = Fx_1*4.44822+Fx_2*4.44822+Fx_3*4.44822+Fx_4*4.44822;

        Ax_brake_in = (Fx_tire-fdrag)/(m_total*g);

    end
    Ax_brake_out(i) = Ax_brake_in*g;
    
end

P1 = polyfit(v_range,Ay_out,3);
P2 = polyfit(v_range,Ax_drive_out,10);
P3 = polyfit(v_range,Ax_brake_out,3);

Ay       = @(x) P1(1).*x.^3 + P1(2).*x.^2 + P1(3).*x + P1(4);
Ax_drive = @(x) P2(1).*x.^10 + P2(2).*x.^9 + P2(3).*x.^8 + P2(4).*x.^7 + P2(5).*x.^6 + P2(6).*x.^5 + P2(7).*x.^4 + P2(8).*x.^3 + P2(9).*x.^2 + P2(10).*x.^1 + P2(11);
Ax_brake = @(x) P3(1).*x.^3 + P3(2).*x.^2 + P3(3).*x + P3(4);

r_turn    = v_range.^2./Ay_out;
r_inside  = r_turn - (track/2);
r_outside = r_turn + (track/2);

a = l*(1-wdf);

steer_angle_inside        = atand(a./r_inside );
steer_angle_outside       = atand(a./r_outside);
steer_angle_inside_wslip  = steer_angle_inside  + alpha_out(:,1)';
steer_angle_outside_wslip = steer_angle_outside + alpha_out(:,2)';

fprintf(' complete.\n')

if ploton
    % Used to check model fit
    figure
    plot(v_range,Ay_out,'bx',v_range,Ay(v_range),'r')
    grid on
    title('Cornering Model')
    xlabel('Velocity (m/s)')
    ylabel('Lateral Acceleration (m/s2)')

    figure
    plot(v_range,Ax_drive_out,'bx',v_range,Ax_drive(v_range),'r')
    grid on
    title('Driving Model')
    xlabel('Velocity (m/s)')
    ylabel('Longitudinal Acceleration (m/s2)')

    figure
    plot(v_range,Ax_brake_out,'bx',v_range,Ax_brake(v_range),'r')
    grid on
    title('Braking Model')
    xlabel('Velocity (m/s)')
    ylabel('Longitudinal Acceleration (m/s2)')
end
