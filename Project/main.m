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
% Installation Test
yalmip('version')
sprintf('The Project files are successfully installed')
% Other parameters
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
% Defines horizon 
N = 20;
% Defines sdpvars
x = sdpvar(10*ones(1,N),ones(1,N));
u = sdpvar(3*ones(1,N-1),ones(1,N-1));
y = sdpvar(3*ones(1,N),ones(1,N));
d = sdpvar(3*ones(1,N),ones(1,N));
% Defines constraints over horizon as well as objective function : 
% objective(y,yredf) = sum_i ((y_i-yref(i))'*R*(y_i-yref(i))
% ( no terminal weight nor terminal set ) 
R = 10*eye(3);
constraints = [];  
objective   = 0;
options = sdpsettings('verbose',0,'solver','gurobi');   % Use Gurobi solver
for i=2:N
    constraints = [constraints, x{i} == A*x{i-1}+ Bu*u{i-1} + Bd*d{i-1}];
    constraints = [constraints, y{i} == C*x{i}];
    constraints = [constraints, Hy*y{i} <= hy];
    constraints = [constraints, Hu*u{i-1} <= hu];
    objective   = objective + (y{i}-yRef(1,:))'*R*(y{i}-yRef(1,:));
end
controller = optimizer(constraints,objective,options,{x{1},[d{:}]},[u{:}]);


%% Section 1: tracking MPC
% Runs simulation 
[~,yt,ut,t] = simBuild(controller,576-N,@shiftPred,N,1);
% Plots results
figure; hold on;
t1  = plot(t,yt(1,:),'LineWidth',1.5,'Color','b');
t2  = plot(t,yt(2,:),'LineWidth',1.5,'Color','r');
t3  = plot(t,yt(3,:),'LineWidth',1.5,'Color','g');
ref = plot(t,24*ones(size(t)),'--','LineWidth',1,'Color',[0.5 0.5 0.5]);
con = plot(t,22*ones(size(t)),'-.','LineWidth',1.2,'Color','k');
plot(t,26*ones(size(t)),'-.','LineWidth',1.2,'Color','k');
legend([t1,t2,t3,ref,con],{'Room 1 temperature','Room 2 temperature','Room 3 temperature','Target temperature','Temperature hard constraints'});
title('Output tracking - building room temperature');
xlabel('Time steps'); ylabel('Temperature (C)');
figure; hold on;
u1  = stairs(t,ut(1,:),'LineWidth',1.5,'Color','b');
u2  = stairs(t,ut(2,:),'LineWidth',1.5,'Color','r');
u3  = stairs(t,ut(3,:),'LineWidth',1.5,'Color','g');
con = plot(t,zeros(size(t)),'-.','LineWidth',1.5,'Color','k');
plot(t,15*ones(size(t)),'-.','LineWidth',1.5,'Color','k');
xlabel('Time step'); ylabel('Power Input (kW');
legend([u1,u2,u3,con],{'Input Room 1','Input Room 2','Input Room 3','Inputs constraints'});
title('MPC Control Inputs');


%% Section 2: economic MPC and soft constraints

% define horizon 
N = 30;
% defines sdpvars
x   = sdpvar(10*ones(1,N),ones(1,N));
u   = sdpvar(3*ones(1,N-1),ones(1,N-1));
y   = sdpvar(3*ones(1,N),ones(1,N));
d   = sdpvar(3*ones(1,N),ones(1,N));
eps = sdpvar(6*ones(1,N), ones(1,N)); % soft constraints 

% define constraints over horizon as well as objective function 
c       = 0.2*ones(3,1); % $/kWh (power cost)
R_eps   = 1*eye(6,6);      % soft constraints penalty

constraints = [];  
objective   = 0;
options = sdpsettings('verbose',1,'solver','gurobi');   % Use Gurobi solver
for i=2:N
    constraints = [constraints, x{i} == A*x{i-1}+ Bu*u{i-1} + Bd*d{i-1}];
    constraints = [constraints, y{i} == C*x{i}];
    constraints = [constraints, Hy*y{i} <= hy + eps{i}];
    constraints = [constraints, eps{i} >= zeros(6,1)];
    constraints = [constraints, Hu*u{i-1} <= hu];
    objective = objective + c'*u{i-1} + eps{i}'*R_eps*eps{i};
end
controller = optimizer(constraints,objective,options,{x{1},[d{:}]},[u{:}]);

simBuild(controller,500,@shiftPred,N,1);


%% Section 3: economic, soft constraints, and variable cost

%fill in here

%% Section 4 : Night setbacks

%fill in here

%% Section 5 : Battery coupled with the building

%fill in here