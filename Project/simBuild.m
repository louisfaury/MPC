%INPUTS:
%  controller - controller (optimizer instance) - the parameter order suggested for the controller is:
%           [x ; xb; d_pred(:) ; cp(:) ; sb(:)]            for economic MPC, where d_pred is the prediction of disturbance input d, cp is the electricity price vector, and sb is the night-setback offset, over the prediction horizon          
%  T - simulation time (in time-steps)
%  fhandle - function handle to the shiftPred function. The input to this
%               function is the time-step and the outputs of this function are the
%               predictions of disturbance d, electricity price cp, and the night-setback offset over the prediction horizon for the given time step. For
%               more details, check the documentation of this function.
%  N - Prediction Horizon of your MPC controller
%  option - 1 for simulation without variable cost and night-setbacks, 2 for variable cost, but no night-setbacks, and 3 for both variable cost and night setbacks

%OUTPUTS:
% xt - state as a function of time
% yt - output as a function of time
% ut - input as a function of time
% t - time (time-steps)




function [ xt, yt, ut, cpt, sbt, t ] = simBuild( controller, T, fhandle, N, option)

load building.mat;
load battery.mat;
% Parameters of the Building Model
A = ssM.A;
Bu = ssM.Bu;
Bd = ssM.Bd;
C = ssM.C;
Ts = ssM.timestep;

% Parameters of the Storage Model
a = ssModel.A;
b = ssModel.Bu; 


x = x0red;

nx = length(A);
nu = size(Bu,2);
nd = size(Bd,2);
ny = size(C,1);


xt = zeros(nx,T);   % T is the length of simulation in samples
yt = zeros(ny,T);
ut = zeros(nu,T);
t = zeros(1,T);
sbMax = zeros(1,T);
sbMin = zeros(1,T);
cpt = zeros(1,T);

%% Simulating the system and the controller
for i = 1:T
[d_pred, cp, sb] = fhandle(i, N);
if option == 1          % No night-setbacks and no variable cost (example)
[U, id] = controller{x,d_pred};

elseif option == 2      % Variable cost, but no night-setbacks
[U, id] = controller{x, d_pred, repmat(cp, 3, 1)};           % this is the suggested form for the controller : you can change it provided buildSim.m is also accordingly changed
    cpt(:,i) = cp(1,1);
elseif option == 3      % Variable cost and night-setbacks
[U, id] = controller{x, d_pred, repmat(cp, 3, 1), repmat(sb, 3, 1)};     % this is the suggested form for the controller : you can change it provided buildSim.m is also accordingly changed
    cpt(:,i) = cp(1,1);
    sbMax(1,i) = sb(1,1);
    sbMin(1,i) = -sb(2,1);
end

xt(:,i) = x;
ut(:,i) = U(1:nu,1);
yt(:,i) = C*x;
t(1,i) = i;
sbt = [sbMax(1,:)',sbMin(1,:)'];

disp(['Iteration ' int2str(i)])
yalmiperror(id)

x = A*x + Bu*U(1:nu,1) + Bd*d_pred(:,1);

end

end

