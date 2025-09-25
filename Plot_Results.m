%% Plot_Results.m
% To be called by LTS

if batch == 0 % Single car setup
    
    avg_vel = sum(speed.*dt/max(time));
    
    % Print Results
    fprintf('Autocross Laptime = %5.3f s\n',laptime)
    fprintf('Endurance Time = \t%5.3f s\n',endurotime)
    fprintf('Skidpad Time = \t\t%5.3f s\n',skidpadtime)
    fprintf('Accel Time = \t\t%5.3f s\n',acceltime)
%     fprintf('Fuel Used = \t\t%4.3f gal\n',fuelused*.264) % Converting from L to gal fuck you steve
    fprintf('Average Velocity = \t%5.3f m/s\n\n',avg_vel)

    % time vs velocity plot
    figure
    scatter(time,speed,2,throttle,'o')
    grid on
    colormap jet
    title('Time vs Speed')
    xlabel('Time (s)')
    ylabel('Speed (m/s)')
    
     % Slip angle vs Velocity
    figure
    hold on
    plot(v_range,alpha_out(:,1))
    plot(v_range,alpha_out(:,2))
    plot(v_range,alpha_out(:,3))
    plot(v_range,alpha_out(:,4))
    legend('Left Front','Right Front','Left Rear','Right Rear')
    title('Slip angle vs Speed')
    xlabel('Speed (m/s)')
    ylabel('Slip Angle (degrees)')
    
    % Steering Angle vs Velocity
    figure
    hold on
    plot(v_range,steer_angle_inside,'--')
    plot(v_range,steer_angle_outside,'--')
    plot(v_range,steer_angle_inside_wslip)
    plot(v_range,steer_angle_outside_wslip)
%     plot(v_range,alpha_out(:,3))
%     plot(v_range,alpha_out(:,4))
    legend('Left Front','Right Front','Left Front','Right Front')
    title('Steer Angle vs Speed')
    xlabel('Speed (m/s)')
    ylabel('Steer Angle (degrees)')
    
elseif batch == 1
    
    % Lap time plot
    figure
    plot(ind_var1,laptime)
    grid on
    title('Lap Time')
    xlabel(var1)
    ylabel('Time (s)')

    % Accel time plot
    figure
    plot(ind_var1,acceltime)
    grid on
    title('Accel Time')
    xlabel(var1)
    ylabel('Accel Time (s)')
    
    % Endurance time plot
    figure
    plot(ind_var1,endurotime)
    grid on
    title('Endurance Time')
    xlabel(var1)
    ylabel('Endurance Time (s)')

    % Endurance points plot
    figure
    plot(ind_var1,enduro_pts)
    grid on
    title('Endurance Points')
    xlabel(var1)
    ylabel('Endurance Points')
    
    % Dynamic event points plot
    figure
    plot(ind_var1,total_pts)
    grid on
    title('Dynamic Event Points')
    xlabel(var1)
    ylabel('Points')
    
    % Endurance fuel usage plot
    %figure
    %plot(ind_var1,fuelused)
    %grid on
    %title('Endurance Fuel Usage')
    %xlabel(var1)
    %ylabel('Fuel (L)')

elseif batch == 2
    
    % Accel Time isoline plot
    figure
    [c,h] = contourf(ind_var2,ind_var1,acceltime,10);
    clabel(round(c),h)
    title('Accel Time')
    xlabel(var2)
    ylabel(var1)
    zlabel('Accel Time (s)')
    d = colorbar;
    ylabel(d, 'Accel Time (s)')
    grid on
    colormap jet
    %savefig(gcf, 'Accel');
    
    % Skidpad Time isoline plot
    figure
    [c,h] = contourf(ind_var2,ind_var1,skidpadtime,10);
    clabel(round(c),h)
    title('Skidpad Time')
    xlabel(var2)
    ylabel(var1)
    zlabel('Skidpad Time (s)')
    d = colorbar;
    ylabel(d, 'Skidpad Time (s)')
    grid on
    colormap jet
    %savefig(gcf, 'Skidpad');
    
    % Laptime isoline plot
    figure
    [c,h] = contourf(ind_var2,ind_var1,laptime,10);
    clabel(round(c),h)
    title('Lap Time')
    xlabel(var2)
    ylabel(var1)
    zlabel('Lap Time (s)')
    d = colorbar;
    ylabel(d, 'Lap Time (s)')
    grid on
    colormap jet
    %savefig(gcf, 'Laptime');
    
    % Autocross score isoline plot
    figure
    [c,h] = contourf(ind_var2,ind_var1,autox_pts,10);
    clabel(c,h)
    title('Autocross Score')
    xlabel(var2)
    ylabel(var1)
    zlabel('Score')
    d = colorbar;
    ylabel(d, 'Score')
    grid on
    colormap jet
    %savefig(gcf, 'Autox');

    % Endurance time isoline plot
    figure
    [c,h] = contourf(ind_var2,ind_var1,endurotime,10);
    clabel(c,h)
    title('Endurance Time')
    xlabel(var2)
    ylabel(var1)
    zlabel('Time (s)')
    d = colorbar;
    ylabel(d, 'Lap Time (s)')
    grid on
    colormap jet
    %savefig(gcf, 'Endurance');

    % Endurance score isoline plot
    figure
    [c,h] = contourf(ind_var2,ind_var1,enduro_pts,10);
    clabel(c,h)
    title('Endurance Score')
    xlabel(var2)
    ylabel(var1)
    zlabel('Score')
    d = colorbar;
    ylabel(d, 'Score')
    grid on
    colormap jet
    %savefig(gcf, 'Endurance Score');
    
    % Dynamic event score isoline plot
    figure
    [c,h] = contourf(ind_var2,ind_var1,total_pts,10);
    clabel(c,h)
    title('Dynamic Event Score')
    xlabel(var2)
    ylabel(var1)
    zlabel('Score')
    d = colorbar;
    ylabel(d, 'Score')
    grid on
    colormap jet
    %savefig(gcf, 'Dynamic Score');
end