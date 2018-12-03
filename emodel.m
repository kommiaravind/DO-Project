function [DyF]=emodel(inst_vel)
%Rolling Resistance
%c = 0.005 + (1 / p) (0.01 + 0.0095 (v / 100)2)
v=inst_vel;
rho_tire = 4.5; %tire presuure in bar
c = 0.005 + (0.01 + 0.0095 .*((0.01.*v).^2))./(rho_tire);
%Fr = c W
m=14000;%kg
W = m * 9.81;
F_r = c*W;

%Grade Dependent force
theta = 0;
F_gr = W * sin(theta);

% Drag Force
%Fd = Cd * A *?* v2/2
Cd = 0.4;
A = 2; %5m/s^2
rho_air = 1.25; %air resistance in kg/m^3
F_d = (Cd.*A.*rho_air.*(v.^2))/2;

%Combined Resitance due to all the forces
DyF = F_r+F_gr+F_d;
end