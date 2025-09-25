function [ max_Fy,alpha ] = Fy_max( Fz,SA_max,res )
global tire_coeff_lat
global mulat

tireFy = @(Fz,SA) pacejka_fun_93(tire_coeff_lat,[Fz SA mulat]);

SA = 0:res:SA_max;
for i = 1:length(SA)
    Fy(i) = tireFy(Fz,SA(i));
end
[max_Fy,I] = max(Fy);
alpha = SA(I);

end
