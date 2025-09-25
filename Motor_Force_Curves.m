% Motor_Force_Curves
% To be called by Ass_2_2.m

Engine_Text = fopen(Motorcode);
%out = textscan(Engine_Text,'%s');
%text = string(out{1,1});
values = csvread(Motorcode,3,0);
engine_spd = values(:,1)'; %rpm
power = values(:,2)'; %hp
torque = values(:,3)'; %ft.lbs
%engine_name = strjoin([text(1) text(2) text(3) text(4) text(5) text(6)]);
power = power.* 0.745699872 * power_coeff; %kW
torque = torque.* 1.35582 * power_coeff; %N.m

tol = engine_spd(1)-engine_spd(2);

for ii = 1:length(v_range)
    v = v_range(ii);
    rpm(ii) = round2((v*finaldrive*(60/(2*pi))/r_tire),tol); %rpm
    if rpm(ii) > rpm_limit
        Fx_engine(ii) = 0;
    else
    ind = find(engine_spd <= rpm(ii));
    Fx_engine(ii) = torque(ind(1))*finaldrive/r_tire; %N
    end
end