function Y = pacejka_fun_93(beta,in)

% in = [Fz, X]
Fz = in(1);
X  = in(2);
mu = in(3);

%beta = [C,c1,c2,c3,c4,c5,c6,c7,c8,dE,SH,SV]
C = beta(1);
c1 = beta(2);
c2 = beta(3);
c3 = beta(4);
c4 = beta(5);
c5 = beta(6);
c6 = beta(7);
c7 = beta(8);
c8 = beta(9);
dE = beta(10);
SH = beta(11);
SV = beta(12);

D = mu.*(c1*Fz.^2+c2*Fz); 
BCD = c3*sind(c4*atand(c5*Fz)); 
B = BCD./(C.*D);
x = X + SH;
E = (c6*Fz.^2+c7.*Fz+c8)+dE*sign(x);
y = D.*sind(C.*atand(B.*x-E.*(B.*x-atand(B.*x)))); 
Y0 = y + SV;
Y0(isnan(Y0)) = 0;
Y = Y0;

end



