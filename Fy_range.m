function [ Fy ] = Fy_range( Fz,SA_range )
global tire_coeff_lat
global mulat

tireFy = @(Fz,SA) pacejka_fun_93(tire_coeff_lat,[Fz SA mulat]);


for i = 1:length(SA_range)
    Fy(i) = tireFy(Fz,SA_range(i));
end

end
