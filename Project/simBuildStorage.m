%INPUTS:
%  controller - controller (optimizer instance) - the parameter order suggested for the controller is:
%           [x ; xb; d_pred(:) ; cp(:) ; sb(:)]            for economic MPC, where d_pred is the prediction of disturbance input d, cp is the electricity price vector, and sb is the night-setback offset, over the prediction horizon          
%  T - simulation time (in time-steps)
%  fhandle - function handle to the shiftPred function. The input to this
%               function is the time-step and the outputs of this function are the
%               predictions of disturbance d, electricity price cp, and the night-setback offset over the prediction horizon for the given time step. For
%               more details, check the documentation of this function.
%  N - Prediction Horizon of your MPC controller
%

%OUTPUTS:
% xt - state as a function of time
% yt - output as a function of time
% ut - input as a function of time
% t - time (time-steps)
% et - electricity input to the battery model as a function of time
% xbt - State of the battery storage as a function of time


function [ xt, yt, ut, t, cpt, sbt, et, xbt, vt] = simBuildStorage(controller, T, fhandle, N)
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
xb = 0;

nx = length(A);
nu = size(Bu,2);
nd = size(Bd,2);
ny = size(C,1);


xt = zeros(nx,T);   % T is the length of simulation in samples
yt = zeros(ny,T);

ut = zeros(nu,T);
t = zeros(1,T);


et = zeros(1,T);
xbt = zeros(1,T);
vt = zeros(1,T);

cpt = zeros(1,T);
sbMax = zeros(1,T);
sbMin = zeros(1,T);


for i = 1:T
[d_pred, cp, sb] = fhandle(i, N);
[U, id] = controller{x, xb, d_pred, cp, repmat(sb, 3, 1)}; % this is the suggested form for the controller : you can change it provided buildSim.m is also accordingly changed

xt(:,i) = x;
ut(:,i) = U(1:nu,1);
et(:,i) = U(end,1);
vt(:,i) = U(end-1,1);
xbt(:,i) = xb;

cpt(:,i) = cp(1,1);
sbMax(:,i) = sb(1,1);
sbMin(:,i) = -sb(2,1);
sbt = [sbMax(1,:)',sbMin(1,:)'];

yt(:,i) = C*x;
t(1,i) = i;

disp(['Iteration ' int2str(i)]);
yalmiperror(id);


x = A*x + Bu*ut(:,i) + Bd*d_pred(:,1);

xb = a*xb + b*[U(nu+1,1)];

end
end

