%INPUT:
%  t - simulation time-step
%  N - Prediction Horizon of your MPC controller

%OUTPUTS:
% pred - prediction of the disturbance input, over the MPC prediction horizon
% cp - shifted price of electricity consumption over the MPC prediction horizon, at time step t
% sb - shifted comfort constraint off-sets over the MPC prediction horizon, at time step t


function [pred, cp, sb] = shiftPred(t, N)

load building.mat;

% Define constants
highPrice = 0.2;    % high energy cost
lowPrice  = 0.04;   % low energy cost

% variables initialization
cp = highPrice*(size(1, N));

%% Disturbance Prediction
pred = refDist(:,t:t+N-1);


%% Variable Price Prediction
futureHours = t + 0:N-1;
% list of index where price is low
listLow         = find((mod(futureHours, 72)>=1.*mod(futureHours, 72)<31) ==1);
listLow         = [listLow , find((mod(futureHours, 72)>=49.* mod(futureHours, 72)<62) == 1)];
cp(1,listLow)   = lowPrice; 


%replace in section 3

%% Night-Setback Prediction
sb = 0;

%replace in section 4


end

