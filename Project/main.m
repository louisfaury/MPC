clc;
close all;
yalmip('clear')
clear;

%% MODEL DATA
load building.mat;
load battery.mat;
% Parameters of the Building Model
A   = ssM.A;
Bu  = ssM.Bu;
Bd  = ssM.Bd;
C   = ssM.C;
Ts  = ssM.timestep;
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
% Hxb*y <= hxb
Hxb = [1, -1]'; hxb = [20 0]';
% Hv*v  <= hv
Hv = [1, -1]'; hv = [20 20]';
% reference 
yRef = 24*ones(3,1);
% arrays
inputs = zeros(3,1);
output = zeros(3,1);
dist   = zeros(3,1);


%% CONTROLLER DESIGN (Setting-up MPC optimizer)
% % Defines horizon 
% N = 20;
% % Defines sdpvars
% x = sdpvar(10*ones(1,N),ones(1,N));
% u = sdpvar(3*ones(1,N-1),ones(1,N-1));
% y = sdpvar(3*ones(1,N),ones(1,N));
% d = sdpvar(3*ones(1,N),ones(1,N));
% % Defines constraints over horizon as well as objective function : 
% % objective(y,yredf) = sum_i ((y_i-yref(i))'*R*(y_i-yref(i))
% % ( no terminal weight nor terminal set ) 
% R = 10*eye(3);
% constraints = [];  
% objective   = 0;
% options = sdpsettings('verbose',0,'solver','gurobi');   % Use Gurobi solver
% for i=2:N
%     constraints = [constraints, x{i} == A*x{i-1}+ Bu*u{i-1} + Bd*d{i-1}];
%     constraints = [constraints, y{i} == C*x{i}];
%     constraints = [constraints, Hy*y{i} <= hy];
%     constraints = [constraints, Hu*u{i-1} <= hu];
%     objective   = objective + (y{i}-yRef(1,:))'*R*(y{i}-yRef(1,:));
% end
% controller = optimizer(constraints,objective,options,{x{1},[d{:}]},[u{:}]);
% 
% %% SECTION 1: tracking MPC
% % Runs simulation 
% [~,yt,ut,~,~,t] = simBuild(controller,576-N,@shiftPred,N,1);
% % Plots results
% plots(t,yt,ut,0,0,0,0,0,1)


%% SECTION 2: economic MPC and soft constraints
% % Defines horizon 
% N = 20;
% % Defines sdpvars
% x   = sdpvar(10*ones(1,N),ones(1,N));
% u   = sdpvar(3*ones(1,N-1),ones(1,N-1));
% y   = sdpvar(3*ones(1,N),ones(1,N));
% d   = sdpvar(3*ones(1,N),ones(1,N));
% eps = sdpvar(6*ones(1,N), ones(1,N)); % soft constraints 
% % Defines constraints over horizon as well as objective function 
% c       = 0.2*ones(3,1);    % $/kWh (power cost)
% R_eps   = 0.1*eye(6,6);     % soft constraints penalty
% constraints = [];  
% objective   = 0;
% options = sdpsettings('verbose',1,'solver','gurobi');   % Use Gurobi solver
% for i=2:N
%     constraints = [constraints, x{i} == A*x{i-1}+ Bu*u{i-1} + Bd*d{i-1}];
%     constraints = [constraints, y{i} == C*x{i}];
%     constraints = [constraints, Hy*y{i} <= hy + eps{i}];
%     constraints = [constraints, eps{i} >= zeros(6,1)];
%     constraints = [constraints, Hu*u{i-1} <= hu];
%     objective = objective + c'*u{i-1} + eps{i}'*R_eps*eps{i};
% end
% controller = optimizer(constraints,objective,options,{x{1},[d{:}]},[u{:}]);
% % Runs simulation
% [~,yt,ut,~,~,t] = simBuild(controller,576-N,@shiftPred,N,1);
% % Plots results
% plots(t,yt,ut,0,0,0,0,0,1);

%% SECTION 3: economic, soft constraints, and variable cost
% % Define shorizon 
%N = 30;
% % Defines sdpvars
% x   = sdpvar(10*ones(1,N),ones(1,N));
% u   = sdpvar(3*ones(1,N-1),ones(1,N-1));
% y   = sdpvar(3*ones(1,N),ones(1,N));
% d   = sdpvar(3*ones(1,N),ones(1,N));
% eps = sdpvar(6*ones(1,N), ones(1,N));       % soft constraints 
% cp  = sdpvar(3*ones(1,N-1),ones(1,N-1));    % variable cost
% 
% % Define constraints over horizon as well as objective function 
% R_eps       = 1*eye(6,6);                                     % soft constraints penalty
% constraints = [];  
% objective   = 0;
% options     = sdpsettings('verbose',1,'solver','+gurobi');   % Use Gurobi solver
% for i=2:N
%     constraints = [constraints, x{i} == A*x{i-1}+ Bu*u{i-1} + Bd*d{i-1}];
%     constraints = [constraints, y{i} == C*x{i}];
%     constraints = [constraints, Hy*y{i} <= hy + eps{i}];
%     constraints = [constraints, eps{i} >= zeros(6,1)];
%     constraints = [constraints, Hu*u{i-1} <= hu];
%     objective = objective + cp{i-1}'*u{i-1} + eps{i}'*R_eps*eps{i};
% end
% controller = optimizer(constraints,objective,options,{x{1},[d{:}], [cp{:}]},[u{:}]);
% % Runs simulation
% [~,yt,ut,cpt,~,t] = simBuild(controller,576-N,@shiftPred,N,2);
% % Plots results
% plots(t,yt,ut,cpt,0,0,0,0,2);

%% SECTION 4 : Night setbacks
% % Defines horizon 
% N = 30;
% % Defines sdpvars
% x   = sdpvar(10*ones(1,N),ones(1,N));
% u   = sdpvar(3*ones(1,N-1),ones(1,N-1));
% y   = sdpvar(3*ones(1,N),ones(1,N));
% d   = sdpvar(3*ones(1,N),ones(1,N));
% eps = sdpvar(6*ones(1,N), ones(1,N));       % soft constraints 
% cp  = sdpvar(3*ones(1,N-1),ones(1,N-1));    % variable cost
% sb  = sdpvar(6*ones(1,N), ones(1,N));       % night setbacks
% 
% % Defines constraints over horizon as well as objective function 
% R_eps   = 1*eye(6,6);      % soft constraints penalty
% 
% constraints = [];  
% objective   = 0;
% options = sdpsettings('verbose',1,'solver','+gurobi');   % Use Gurobi solver
% for i=2:N
%     constraints = [constraints, x{i} == A*x{i-1}+ Bu*u{i-1} + Bd*d{i-1}];
%     constraints = [constraints, y{i} == C*x{i}];
%     constraints = [constraints, Hy*y{i} <= sb{i} + eps{i}];
%     constraints = [constraints, eps{i} >= zeros(6,1)];
%     constraints = [constraints, Hu*u{i-1} <= hu];
%     objective = objective + cp{i-1}'*u{i-1} + eps{i}'*R_eps*eps{i};
% end
% controller = optimizer(constraints,objective,options,{x{1},[d{:}], [cp{:}], [sb{:}]},[u{:}]);
% % Runs simulation 
% [~,yt,ut,cpt,sbt,t] = simBuild(controller,576-N,@shiftPred,N,3);
% % Plots results 
%  plots(t,yt,ut,cpt,sbt,0,0,0,3);
 
%% SECTION 5 : Battery coupled with the building
% % Defines horizon 
% N = 30;
% % Defines sdpvars
% x       = sdpvar(10*ones(1,N),ones(1,N));
% u       = sdpvar(3*ones(1,N-1),ones(1,N-1));
% y       = sdpvar(3*ones(1,N),ones(1,N));
% d       = sdpvar(3*ones(1,N),ones(1,N));
% eps     = sdpvar(6*ones(1,N), ones(1,N));       % soft constraints 
% cp      = sdpvar(ones(1,N-1),ones(1,N-1));      % variable cost for e_k this time
% sb      = sdpvar(6*ones(1,N), ones(1,N));       % night setbacks
% xb      = sdpvar(ones(1,N), ones(1,N));         % battery storage
% e       = sdpvar(ones(1,N-1), ones(1,N-1));     % energy got from the grid
% v       = sdpvar(ones(1,N-1), ones(1,N-1));
% % Defines constraints over horizon as well as objective function 
% R_eps   = 1*eye(6,6);      % soft constraints penalty
% constraints = [];  
% objective   = 0;
% options = sdpsettings('verbose',1,'solver','+gurobi');   % Use Gurobi solver
% for i=2:N
%     constraints = [constraints, x{i} == A*x{i-1}+ Bu*u{i-1} + Bd*d{i-1}];
%     constraints = [constraints, y{i} == C*x{i}];
%     constraints = [constraints, Hy*y{i} <= sb{i} + eps{i}];
%     constraints = [constraints, eps{i} >= zeros(6,1)];
%     constraints = [constraints, Hu*u{i-1} <= hu];
%     constraints = [constraints, e{i-1} >= 0];
%     constraints = [constraints, v{i-1} == e{i-1} - ones(1,3)*u{i-1}];
%     constraints = [constraints, xb{i} == a*xb{i-1} + b*v{i-1}];
%     constraints = [constraints, Hxb*xb{i} <= hxb];
%     constraints = [constraints, Hv*v{i-1} <= hv];
%     objective = objective + cp{i-1}*e{i-1} + eps{i}'*R_eps*eps{i};
% end
% controller = optimizer(constraints,objective,options,{x{1}, xb{1}, [d{:}], [cp{:}], [sb{:}]},{[u{:}; v{:}; e{:}]});
% % Runs simulation
% [xt,yt,ut,t,cpt,sbt,et,xbt,vt] = simBuildStorage(controller, 576-N, @shiftPred, N);
% % Plots results 
%  plots(t,yt,ut,cpt,sbt,xbt,et,vt,4);

 %% SECTION 6 : several simulations with different battery properties 
 % Varying the battery's dissipation factor
 alpha_array = [0.25 0.5 0.75 1];
 cost        = zeros(1, size(alpha_array, 2));
 % Defines horizon
 N = 30;
 % Defines sdpvars
 x       = sdpvar(10*ones(1,N),ones(1,N));
 u       = sdpvar(3*ones(1,N-1),ones(1,N-1));
 y       = sdpvar(3*ones(1,N),ones(1,N));
 d       = sdpvar(3*ones(1,N),ones(1,N));
 eps     = sdpvar(6*ones(1,N), ones(1,N));       % soft constraints
 cp      = sdpvar(ones(1,N-1),ones(1,N-1));      % variable cost for e_k this time
 sb      = sdpvar(6*ones(1,N), ones(1,N));       % night setbacks
 xb      = sdpvar(ones(1,N), ones(1,N));         % battery storage
 e       = sdpvar(ones(1,N-1), ones(1,N-1));     % energy got from the grid
 v       = sdpvar(ones(1,N-1), ones(1,N-1));
 % Defines constraints over horizon as well as objective function
 R_eps   = 1*eye(6,6);      % soft constraints penalty
 options = sdpsettings('verbose',1,'solver','+gurobi');   % Use Gurobi solver

 k=1;
 for alpha = alpha_array
     objective   = 0;
     constraints = [];
     for i=2:N
         constraints = [constraints, x{i} == A*x{i-1}+ Bu*u{i-1} + Bd*d{i-1}];
         constraints = [constraints, y{i} == C*x{i}];
         constraints = [constraints, Hy*y{i} <= sb{i} + eps{i}];
         constraints = [constraints, eps{i} >= zeros(6,1)];
         constraints = [constraints, Hu*u{i-1} <= hu];
         constraints = [constraints, e{i-1} >= 0];
         constraints = [constraints, v{i-1} == e{i-1} - ones(1,3)*u{i-1}];
         constraints = [constraints, xb{i} == alpha*xb{i-1} + b*v{i-1}];
         constraints = [constraints, Hxb*xb{i} <= hxb];
         constraints = [constraints, Hv*v{i-1} <= hv];
         objective = objective + cp{i-1}*e{i-1} + eps{i}'*R_eps*eps{i};
     end
     controller = optimizer(constraints,objective,options,{x{1}, xb{1}, [d{:}], [cp{:}], [sb{:}]},{[u{:}; v{:}; e{:}]});
     % Runs simulation
     [xt,yt,ut,t,cpt,sbt,et,xbt,vt] = simBuildStorage(controller, 576-N, @shiftPred, N);
     % Plots results
     plots(t,yt,ut,cpt,sbt,xbt,et,vt,5, hxb(1));
    % Compute total cost in dollars
      cost(1,k) = sum(cpt.*et);
      k=k+1;
 end
 % Disp total costs
 disp('####################')
 disp('')
 disp('ENERGY BILLS:')
 for i=1:size(cost,2)
     disp(strcat('Dissipation factor: ' , num2str(alpha_array(1,i)), '   / Total cost: ', num2str(round(cost(1,i),2)), '$'));
 end
 
 % Varying the battery's strorage capacity
%  hxb_array = [[10 0]',[30 0]',[50,0]',[100,0]'];
%  cost      = zeros(1, size(hxb_array, 2));
%  % Defines horizon
%  N = 30;
%  % Defines sdpvars
%  x       = sdpvar(10*ones(1,N),ones(1,N));
%  u       = sdpvar(3*ones(1,N-1),ones(1,N-1));
%  y       = sdpvar(3*ones(1,N),ones(1,N));
%  d       = sdpvar(3*ones(1,N),ones(1,N));
%  eps     = sdpvar(6*ones(1,N), ones(1,N));       % soft constraints
%  cp      = sdpvar(ones(1,N-1),ones(1,N-1));      % variable cost for e_k this time
%  sb      = sdpvar(6*ones(1,N), ones(1,N));       % night setbacks
%  xb      = sdpvar(ones(1,N), ones(1,N));         % battery storage
%  e       = sdpvar(ones(1,N-1), ones(1,N-1));     % energy got from the grid
%  v       = sdpvar(ones(1,N-1), ones(1,N-1));
%  % Defines constraints over horizon as well as objective function
%  R_eps   = 1*eye(6,6);      % soft constraints penalty
%  options = sdpsettings('verbose',1,'solver','+gurobi');   % Use Gurobi solver
%  
%  k=1;
%  for hxb = hxb_array
%      objective   = 0;
%      constraints = [];
%      for i=2:N
%          constraints = [constraints, x{i} == A*x{i-1}+ Bu*u{i-1} + Bd*d{i-1}];
%          constraints = [constraints, y{i} == C*x{i}];
%          constraints = [constraints, Hy*y{i} <= sb{i} + eps{i}];
%          constraints = [constraints, eps{i} >= zeros(6,1)];
%          constraints = [constraints, Hu*u{i-1} <= hu];
%          constraints = [constraints, e{i-1} >= 0];
%          constraints = [constraints, v{i-1} == e{i-1} - ones(1,3)*u{i-1}];
%          constraints = [constraints, xb{i} == a*xb{i-1} + b*v{i-1}];
%          constraints = [constraints, Hxb*xb{i} <= hxb];
%          constraints = [constraints, Hv*v{i-1} <= hv];
%          objective = objective + cp{i-1}*e{i-1} + eps{i}'*R_eps*eps{i};
%      end
%      controller = optimizer(constraints,objective,options,{x{1}, xb{1}, [d{:}], [cp{:}], [sb{:}]},{[u{:}; v{:}; e{:}]});
%      % Runs simulation
%      [xt,yt,ut,t,cpt,sbt,et,xbt,vt] = simBuildStorage(controller, 576-N, @shiftPred, N);
%      % Plots results
%      plots(t,yt,ut,cpt,sbt,xbt,et,vt,5,hxb(1));
%      % Compute total cost in dollars
%      cost(1,k) = sum(cpt.*et);
%      k=k+1;
%  end
%  
%  % Disp total costs
%  disp('####################')
%  disp('')
%  disp('ENERGY BILLS:')
%  for i=1:size(cost,2)
%      disp(strcat('Storage limit: ' , num2str(hxb_array(1,i)), 'kWh    / Total cost: ', num2str(round(cost(1,i),2)), '$'));
%  end
%  