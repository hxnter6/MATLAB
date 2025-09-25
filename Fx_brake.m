function [ max_Fx ] = Fx_brake( Fz,SR_max, res )
global tire_coeff_lon
global mulon

tire = @(Fz,SR) pacejka_fun_93(tire_coeff_lon,[Fz SR mulon]);

SR = 0:-res:-SR_max;
for i = 1:length(SR)
    Fx(i) = tire(Fz,SR(i));
end
max_Fx = -min(Fx);

end
