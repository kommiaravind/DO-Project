close all; clearvars; clc;

%% Simuation Parameters
L = 1000; % Length of the  road 
l=L/10;
v_max = 65;% Maximum speed
p = 0.30; % Probability of sudden stop
Te = 300; % Effective number of observations
N = 50; % Number of vehicles
v = zeros(l,1); % Allocation
f = zeros(l,1); % Allocation
clear res;
for n=1:l  
    res = trafficsim(v_max,l,n,p,Te);
    v(n) = res.v_mean;
    f(n) = res.flow_mean;
    rho(n) = res.rho;
end
%% Traffic conditions graph
% Velocity vs density
figure()
subplot(2,1,1);
plot(rho,v,'-ro'); 
title('Model for traffic simulation');
xlabel('{\rho}: Density');
ylabel('Mean velocity');
grid on;

% Flow vs density
subplot(2,1,2);
plot(rho,f,'-bo'); 
xlabel('{\rho}: Density');
ylabel('Flow');
grid on;

%% Traffic conditions velocity graph-Drive cycle plot 
clear res;
res = trafficsim(v_max,L,N,p,Te);

% Mean velocity
figure()
plot(res.v)
xlabel('Time(seconds)'); ylabel('Velocity'); title('Velocity');
grid on;

% Accleration
figure()
plot(res.a)
xlabel('Time(seconds)'); ylabel('Acceleration'); title('Acceleration');
grid on

%% Load drive cycle of car ahead
load('drive_cyc.mat')