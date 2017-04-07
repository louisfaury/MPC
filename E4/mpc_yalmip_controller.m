function [uopt] = mpc_yalmip_controller(N,A,B,Q,P,R,x0,Hx,hx,Hu,hu,Xf)
%! computes the next control from the MPC controller thanks to YALMIP !%

% Define optimization variables
u = sdpvar(repmat(1,1,N),repmat(1,1,N));
x = sdpvar(repmat(2,1,N+1),repmat(1,1,N+1));

% Define constraints and objective
con = [];
obj = 0;
for i = 1:N
    con = [con , x{i+1} == A*x{i} + B*u{i}];   % System dynamics
    con = [con , Hx*x{i+1} <= hx];            % State constraints
    con = [con , Hu*u{i} <= hu];              % Input constraints
    obj = obj + x{i}'*Q*x{i} + u{i}'*R*u{i};  % Cost function
end

con = [con, Xf.A*x{N+1} <= Xf.b]; % Terminal constraint
obj = obj + x{N+1}'*P*x{N+1};    % Terminal weight

% Compile the matrices
ctrl = optimizer(con, obj, [],x{1},[u{:}]);
% Can now compute the optimal control input using
uopt = ctrl{x0};

end