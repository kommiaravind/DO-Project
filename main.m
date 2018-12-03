clc;
traffic()
%% Optimization
P=load('drive_cyc.mat');
Q=load('enginedata.mat');
safe_dist=5;%m
mass=14000;%Kg
diatyre=0.6243;%m
FR=5;
ICE_torque=Q.eng_consum_trq;
ICE_speed=Q.eng_consum_spd;
s=Q.eng_consum_fuel;
p=[4.723 2.645 2.47 0.3083 0.9967 0.5345 0.06051 -0.1235 0.415];
func=@(x) (p(1)+p(2).*x(1)+p(3).*x(2)+p(4).*x(1)^2+p(5).*x(1).*x(2)+p(6).*x(2)^2+p(7).*x(1)^3+p(8).*x(1)^2.*x(2)+p(9).*x(1).*x(2)^2);
x0=[100,100];
T=zeros();
initial_vel=zeros(1,length(1:Te));
inst_vel=zeros(1,length(1:Te));

for i=1:length(1:Te)
    Aeq=[1,0];
    Beq=[(P.cyc_mph(i)*1-safe_dist)*mass*(0.5*diatyre)/FR];
    A=[];%res.v(i),-initial_vel(i);-res.v(i),initial_vel(i)
    B=[];%0;5
    ub=[8*max(ICE_torque), max(ICE_speed)];
    lb=[100,min(ICE_speed)];
    [x,fval]=fmincon(func,x0,A,B,Aeq,Beq,lb,ub);
    T(i)=x(1);
    inst_vel(i)=T(i)*FR./(mass*0.5*diatyre);
    mass_flowrate(i)=fval/3.6e9; %KWh to Js
    initial_vel(i)=inst_vel(i);
    d_traveled(i)=initial_vel(i)+inst_vel(i)*0.5;
    x0=x;
end

%Saving optimized drive cycle
time_mph_1=[1:Te]';
cyc_mph_1=inst_vel';
save('drive_cyc_1','time_mph_1','cyc_mph_1');

%Plotting the vehicle drive cycles 
figure()
plot(1:Te,inst_vel,'r'); % vehicle optimized drive cycle 
hold on;
plot(1:Te,P.cyc_mph,'k'); % traffic flow drive cycle 
title('Vehicle drive cycles : traffic flow and Optimized')
xlabel('Time(seconds)'); ylabel('Velocity')
legend('Follower','Leader');
grid on

%Plotting mass flow rate
figure()
plot(mass_flowrate);
title('Mass flow rate overtime');
xlabel('Time(seconds)');ylabel('Mass flow rate');
grid on

%dynamic model energy consumption
[dynF]=emodel(inst_vel);
plot(1:Te,inst_vel);
grid on;