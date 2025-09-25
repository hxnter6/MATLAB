function [v_,t_,throttle] = corner(Ay,r,d,roundoff,dt,t_start)
% Calculates Cornering velocity and time

v = sqrt(Ay*r);
t = d/v;
t = round(t,roundoff);

t_end = t_start+t;
t_ = t_start:dt:t_end;

v_ = ones(1,numel(t_)).*v;
throttle = 0.3.*ones(1,numel(t_)); %0.3 Scaling factor based on injector duty cycle%

end

