clc;
close all;
yalmip('clear')
clear all

%% Model data
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
c = 0.2*ones(3,1); % $/kWh (power cost)
% Installation Test
yalmip('version')
sprintf('The Project files are successfully installed')


%Other parameters
% constraints 
% Hu?u <= hu 
Hu = kron(eye(3),[1;-1]); hu = repmat([15 0]',3,1);
% Hy?y <= huy
Hy = kron(eye(3),[1;-1]); hy = repmat([26 -22]',3,1);
% reference 
yRef = 24*ones(3,1);
% arrays
inputs = zeros(3,1);
output = zeros(3,1);
dist   = zeros(3,1);

%% Controller Design (Setting-up MPC optimizer)
% define horizon 
N = 2;
% defines sdpvars
x = sdpvar(10*ones(1,N),ones(1,N));
u = sdpvar(3*ones(1,N-1),ones(1,N-1));
y = sdpvar(3*ones(1,N),ones(1,N));
d = sdpvar(3*ones(1,N),ones(1,N));

% define constraints over horizon as well as objective function 
% objective(y,yredf) = sum_i ((y_i-yref(i))'*R*(y_i-yref(i))
% no terminal weight nor terminal set 
R = diag([1 2 1]);
constraints = [];  
objective   = 0;
options = sdpsettings('verbose',1,'solver','gurobi');   % Use Gurobi solver
for i=2:N
    constraints = [constraints, x{i} == A*x{i-1}+ Bu*u + Bd*d{i-1}];
    constraints = [constraints, y{i} == C*x{i}];
    constraints = [constraints, Hy*y{i} <= hy];
    constraints = [constraints, Hu*u <= hu];
    objective   = objective + (y{i}-yRef(1,:))'*R*(y{i}-yRef(1,:));
end
controller = optimizer(constraints,objective,options,{x{1},[d{:}]},u);

%% Section 1: tracking MPC

% simBuild(controller,500,@shiftPred,N,1);

% nice plots 

% different paramaters 

% start report 

%% Section 2: economic MPC and soft constraints
% change cost function 
objective = 0; 
for i=1:N-1
    objective = objective + c'*u;
end
controller = optimizer(constraints,objective,options,{x{1},[d{:}]},u);

simBuild(controller,200,@shiftPred,N,1);

%% Section 3: economic, soft constraints, and variable cost

%fill in here

%% Section 4 : Night setbacks

%fill in here

%% Section 5 : Battery coupled with the building

%fill in here