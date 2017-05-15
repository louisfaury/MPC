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
highMax   = 30;     % max value for high bound on y
lowMax    = 26;     % min value for high bound on y
highMin   = 22;     % max value for low bound on y
lowMin    = 18;     % min value for low bound on y


% variables initialization
cp = highPrice*ones(1, N-1);
sb = repmat([highMax -lowMin]',1,N);

%% Disturbance Prediction
pred = refDist(:,t:t+N-1);


%% Variable Price Prediction
futureHours = (t-1)*ones(1, N-1) + (1:N-1);
% list of index where price is low
listLow         = find((mod(futureHours, 72)>=0).*(mod(futureHours, 72)<31) == 1); % between 0:00 to 10:00 between 16:00 to 00:00 
listLow         = [listLow , find((mod(futureHours, 72)>=49) == 1)];               % between 16:00 to 00:00
cp(1,listLow)   = lowPrice; 

%% Night-Setback Prediction
futureHours = (t-1)*ones(1, N) + (1:N);
% list of index where constraints on y are more severe
listHard = find((mod(futureHours, 72)>=25).*(mod(futureHours, 72)<55) == 1); % between 8:00 and 18:00
sb(:, listHard) = repmat([lowMax -highMin]',1,length(listHard));

end

