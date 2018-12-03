function [res,drive_cyc] = trafficsim(v_max,L,N,p,Te)
rng(4); % seed
if (N > L)
    error('** Number of vehicles must be lower or equal than the size of the road **');
end

rho = N/L; % Density
Tr = 10*L; % Relaxation time 
T = Te+Tr; % Total time
% Allocation and initial velocities
keyNA = -999;
v = zeros(T,1);
pos = zeros(1,N);
XV = keyNA*ones(T,L); 

%% Simulation of model
% Initial positions
aux = 1:L;
for j=1:N        
    u = randi([1,length(aux)]);
    pos(j) = aux(u);
    aux(u) = [];
end
% Relocating initial positions
pos = sort(pos);
% Intial condition for matrix time x (velocity at position)
for j=1:N
    XV(1,pos(j)) = 0;
end
% Dynamics
for t=2:T
    % Extract velocities and positions observed at t-1
    vel = XV(t-1,:);
    I = find(vel ~= keyNA);
    vel = vel(I);
    pos = I;
    % Determining distances
    d = [diff(pos)   L-pos(end)+pos(1)];
    d = d - 1;
    % Loop on vehicles
    for j=1:N
        % Acceleration
        v1 = min(vel(j)+1, v_max);
        % Avoiding crashes
        v2 = min(v1, d(j));
        % Random sudden deceleration
        u = rand;
        % Updating velocity
        if (u <p)
            vel(j) = max(v2-1,0);
        else
            vel(j) = v2;
        end
        % New position
        pos(j) = pos(j) + vel(j);
        if (pos(j) > L)
            pos(j) = pos(j) - L;
        end
        % Updating XV matrix
        XV(t,pos(j)) = vel(j);
    end
    v(t) = mean(vel);
end 
v(1:Tr) = [];
XV(1:Tr,:) = [];
v_mean = mean(v); % Global mean
flow_mean = v_mean*rho;

% find and add acceleration 
for n=1:Te-1
    a(n)=v(n+1)-v(n); % dt=1
%     s(n)=res.v(n)+0.5*a(n);
end

%% Adding to structure
res.L = L;
res.N = N;
res.v_max = v_max;
res.p = p;
res.Te = Te;
res.rho = rho;
res.v = v;
res.v_mean = v_mean;
res.flow_mean = flow_mean;
res.XV = XV;
res.a=a;

%% Saving data to file for retrival
cyc_mph=v;
time_mph=[1:Te]';
save('drive_cyc.mat','cyc_mph','time_mph')