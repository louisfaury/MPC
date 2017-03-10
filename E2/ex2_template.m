%%%
% MPC-425, Exercise 2
%%%


function ex2

clc;
clear;
close all;

% Problem:
%  min  0.5 * z' * prob.H * z + prob.q' * z
%  s.t. prob.G * z <= prob.d

%% Choose problem parameters
dim   = 2; % Number of optimization variables
speed = 'slow'; % set to 'fast' for fast convergence, and 'slow' to see what's going on
seed  = ceil(100*rand); % Set to any integer to choose the randomly generated problem

[prob,opt] = setupEx2(dim, speed, seed);

% Change this parameter for exercise 2
 opt.mu      = 5e-3;


%%%%%%%%%%%%%%%%%%% THE BARRIER METHOD %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% THE BARRIER METHOD %%%%%%%%%%%%%%%%%%%

% Initialize
kappa = opt.kappa0;
z     = prob.z0;
zprev = z;

% Outer loop
stats.outerIterations = 0;
stats.innerIterations = 0;
tic
while kappa > opt.epsilon % Stop once the barrier term is small
  stats.outerIterations = stats.outerIterations + 1;
  
  % Inner loop (centering step)
  while 1
    stats.innerIterations = stats.innerIterations + 1;
    if any(prob.d-prob.G*z < 0), error('Current point z is not feasible'); end

    %vvvvvvvvvvvvvvvv YOUR CODE HERE vvvvvvvvvvvvvvvvvvv

    % Compute search direction 
    
    % Computing matrix sum(1/(di-giz)2*gi.Tgi)
    sum1 = (repmat(1./(prob.d-prob.G*z).^2, 1, size(z, 1)).*prob.G)'*prob.G;
    
    
    % Computing vector sum(1/(di-giz)*gi.T)
    sum2 = sum((repmat(1./(prob.d-prob.G*z), 1, size(z, 1)).*prob.G), 1)';
    
    % Solving fullHessian*Dz = -fullGrad
    fullHessian = prob.H + kappa*sum1;
    fullGrad = prob.H*z+prob.q+kappa*sum2;
    Dz = -linsolve(fullHessian,fullGrad);

    %^^^^^^^^^^^^^^^^ YOUR CODE HERE ^^^^^^^^^^^^^^^^^^^

    if dim == 2, plot([z(1);zprev(1)],[z(2);zprev(2)],'k.-','markersize',10); drawnow; end

    t = 1;

    % Reduce t until z+t*Dz is feasible
    % (Backtrack for feasibility)
    % Use opt.beta / opt.alpha as the backtracking parameters

    while t > opt.epsilon && any(prob.G*(z+t*Dz) > prob.d)
      t = opt.beta*t;
    end

    % Reduce t to minimize f(z+t*Dz)
    % Use opt.beta / opt.alpha as the backtracking parameters

    while t > opt.epsilon
      fup   = prob.f(z+t*Dz) + kappa * prob.phi(z+t*Dz); 
      df    = prob.gradF(z)  + kappa * prob.gradPhi(z);
      fdown = prob.f(z)      + kappa * prob.phi(z) + opt.alpha*t*df'*Dz;

      if fup < fdown, break; end
      t=opt.beta*t;
    end

    % Termination condition
    % We stop if we're no longer making any progress, either because the
    % step size t is very small, or the size of the gradient is very small
    if t < opt.epsilon || norm(Dz) < opt.epsilon, break; end
    
    % Slow down convergence so we can see what's going on
    if opt.slow, t = min([t t/(20*norm(Dz)) ]); end
    
    % Take step
    zprev = z;
    z = z + t*Dz;
  end

  if dim == 2, plot(z(1),z(2),'ro'); drawnow; end

  % Decrease barrier parameter
  kappa = kappa * opt.mu;
end
stats.solveTime = toc;

%%%%%%%%%%%%%%%%%%% THE BARRIER METHOD %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% THE BARRIER METHOD %%%%%%%%%%%%%%%%%%%

%%
% Solve the optimization problem and compare
tic
[zopt,fval,flag] = quadprog(prob.H,prob.q,prob.G,prob.d);
stats.quadprogSolveTime = toc;
if flag ~= 1, error('Could not solve optimization problem'); end

fprintf('---------------------\n');
fprintf('Your optimal cost: %f\n', prob.f(z));
fprintf('True optimal cost: %f\n', fval);
fprintf('Error = %f\n\n', abs(prob.f(z) - fval));


if dim==2
  fprintf('---------------------\n');
  fprintf('Your optimal solution: [%.3f %.3f]\n', z(1), z(2));
  fprintf('True optimal solution: [%.3f %.3f]\n', zopt(1), zopt(2));
  fprintf('Norm of error = %f\n\n', norm(z-zopt));
end

fprintf('---------------------\n');
fprintf('Solution statistics:\n');
fprintf('  Number of outer iterations:         %i\n', stats.outerIterations);
fprintf('  Total number of inner iterations: %.2f\n', stats.innerIterations);
fprintf('  Solve time:                         %.2f seconds\n', stats.solveTime);
fprintf('  Matlab solve time:                  %.2f seconds\n', stats.quadprogSolveTime);


end

function [prob, opt] = setupEx2(dim, speed, seed)
%
% Setup a random quadratic program
%

%% Create a random quadratic program with dim variables
prob = randomQP(dim, seed);

%% Plot the problem
if dim == 2
  plotOptimization(prob);
end

%% Barrier method parameters
% Select 'true' for slow convergence, so you can see the central path.
% Select 'false' for standard, fast convergence
opt = setBarrierParameters(strncmp(speed, 'slow', 1));

end

%% Set the optimization parameters
function opt = setBarrierParameters(isSlow)
if isSlow
  % Use these parameters for "slow" convergence, so you can see the central path
  opt.kappa0  = 100;   % Initial value of barrier parameter
  opt.alpha   = 0.02;  % Line search parameters
  opt.beta    = 0.9;
  opt.mu      = 0.8;   % Decrease rate for barrier parameter
  opt.epsilon = 1e-3;  % Optimality tolerance
  opt.slow    = true;
else
  % Use these parameters for "fast" convergence, to solve the optimization problem
  opt.kappa0  = 1;
  opt.alpha   = 0.02;
  opt.beta    = 0.7;
  opt.mu      = 1e-2;
  opt.epsilon = 1e-8;
  opt.slow    = false;
end
end

%% Plot the optimization problem (if it is 2D)
function plotOptimization(prob)
if prob.dim ~= 2
  fprintf('Can only plot two dimensional optimization problems\n');
  return
end
clf; hold on; grid on;
try
  plot(polytope(prob.G,prob.d),struct('shade',0,'linewidth',3))
  %plotPolyLine(polytope(prob.G,prob.d),'k',3);
  
  %[X,Y] = gridPolytope(polytope(prob.G,prob.d),50);
  [B,l,u]=bounding_box(polytope(prob.G,prob.d));
  [X,Y] = meshgrid(linspace(l(1)-0.1,u(1)+0.1,50),linspace(l(2)-0.1,u(2)+0.1,50));
  F = 0*X;
  for i = 1:size(X,1)
    for j = 1:size(X,2)
      z = [X(i,j);Y(i,j)];
      F(i,j) = prob.f(z);
    end
  end
  contour(X,Y,F,20);
catch
  warning('Could not plot the constraints - possibly MPT is not installed?');
end

% Compute optimal solution and plot
try
  [zopt,fval,flag] = quadprog(prob.H,prob.q,prob.G,prob.d);
  if flag ~= 1, error('Could not solve optimization problem'); end
  plot(zopt(1),zopt(2),'ko','markersize',10,'markerfacecolor','k');
  text(zopt(1)+0.05,zopt(2),'Optimal point','fontweight','bold','fontsize',12,'backgroundcolor','w');
  plot(prob.z0(1),prob.z0(2),'ko','markersize',10,'markerfacecolor','k');
  text(prob.z0(1)+0.05,prob.z0(2),'Initial point','fontweight','bold','fontsize',12,'backgroundcolor','w');
catch
  warning('Could not compute the optimal solution - possibly optimization toolbox isn''t installed?');
end

% Compute analytic center and plot
try
  x = sdpvar(size(prob.G,2),1);
  dd = solvesdp([prob.d-prob.G*x >= 0], -geomean(prob.d-prob.G*x));
  if dd.problem ~= 0, error('Could not compute analytic center'); end
  x = double(x);
  plot(x(1),x(2),'ko','markersize',10,'markerfacecolor','k');
  text(x(1)+0.05,x(2),'Analytic center','fontweight','bold','fontsize',12,'backgroundcolor','w');
  axis tight
catch
  warning('Could not compute analytic center - possibly YALMIP isn''t installed?');
end

end

%% Create a random quadratic program
function prob = randomQP(dim, seed)

% Number of optimization variables
prob.dim = dim;

% Set the seed of the random generator so we can get the same 'random'
% problem twice if we want
randn('state',seed);

% Create the problem
%  min  0.5 z'Hz + q'z s.t. Gx <= d
prob.G  = randn(5*dim,dim);
prob.d  = ones(5*dim,1);
prob.H  = randn(dim); prob.H = prob.H * prob.H';
prob.q  = randn(dim,1);

% We've chosen d = 1, so the point 0 is always in the interior of the
% constraints
prob.z0 = zeros(dim,1);

% Helper functions
prob.f       = @(z) 0.5*z'*prob.H*z + prob.q'*z;
prob.gradF   = @(z) prob.H*z + prob.q;
prob.phi     = @(z) sum(-log(prob.d-prob.G*z));
prob.gradPhi = @(z) prob.G'*diag(1./(prob.d-prob.G*z))*ones(length(prob.d),1);

end
