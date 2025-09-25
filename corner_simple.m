function [v_] = corner(Ay,r,d,roundoff,dt,t_start)
% Calculates Cornering velocity and time

v = sqrt(Ay*r);
t = d/v;
t = round(t,roundoff);

t_end = t_start+t;
t_ = t_start:dt:t_end;

v_ = ones(1,numel(t_)).*v;


end

