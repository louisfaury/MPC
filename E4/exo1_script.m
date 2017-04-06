%% MPC class
%% E4 script for exercice 1 

clear all;
clc;
close all; 

% defs 
A = [0.9752 1.4544; -0.0327 0.9315];
B = [0.0248; 0.0327]; 
Hx = [1 0; -1 0; 0 1; 0 -1];
hx = [5; 5; 0.2; 0.2];
Hu = [1;-1] ; 
hu = 1.75*[1;1];
Q = 10*eye(2);
R = 1;
N = 10;
X = Polyhedron(Hx,hx);

%% Algo - question 1
% LQR feedback law
[Klqr,P,~] = dlqr(A,B,Q,R); % [optimal LQR gain, DARE solution]
Vf = @(x) x'*P*x;           % terminal cost 

 % Defines new constraint set 
 convergence = false;                       % convergence flag
 O_lqr = Polyhedron([Hx;-Hu*Klqr],[hx;hu]); % current estimate for O_infinity

 %! ------------------------------------------------------ !%
 %      computing largest invariant set for lqr             %
 %! ------------------------------------------------------ !%
 while (~convergence)
     Op = Polyhedron(O_lqr.A*(A-B*Klqr),O_lqr.b);           % current Omega pre-set
     On = Polyhedron( [Op.A;O_lqr.A] , [Op.b;O_lqr.b] );    % update rule
     convergence = (O_lqr == On);                           % convergence check
     O_lqr = On;                                            % update 
 end
  %! ------------------------------------------------------ !%
  
%  % Results 
%  algo_res.weight = P;
%  algo_res.set = O_lqr;
%  algo_res.controller = -Klqr;
%  
%  % Verifies result with MPT3
%  sys = LTISystem('A',A,'B',B);
%  sys.x.max = [5; 0.2]; sys.x.min = [-5; -0.2];
%  sys.u.max = 1.75; sys.u.min = -1.75;
%  sys.x.penalty = QuadFunction(Q);
%  sys.u.penalty = QuadFunction(R);
%  mpt_res.weight = sys.LQRPenalty.weight;
%  mpt_res.set = sys.LQRSet;
%  mpt_res.controller = sys.LQRGain;
%  
%  % display 
%  disp('Boolean value for equality between sets :')
%  disp(mpt_res.set==algo_res.set);
%  disp('MPT terminal state :');
%  disp(mpt_res.weight); 
%  disp('Our terminal state :');
%  disp(algo_res.weight);
%  disp('MPT terminal gain :');
%  disp(mpt_res.controller); 
%  disp('Our terminal gain :');
%  disp(algo_res.controller);
     
 
 %% Question 2
 % cost function = 0.5*z'Hz + hz with z = (x11,x12, .., x1N, x2N, u0, .., u_(N_1)
 % with constraint (Tz=t) and (Gz<=g)
 x = [3;0];
 max_iter = 30;
 traj = zeros(max_iter,2);
 control = zeros(max_iter,1);
 for i=1:20
     traj(i,:) = x;
    [H,h,G,g,T,t] = mpc_precompute(N,A,B,Q,P,R,x,Hx,hx,Hu,hu,O_lqr);      % precomputes matrix for quadprog minimization
    [zopt, fval, flag] = quadprog(2*H, h, G, g, T, t);                     % 2*H to comply with model : 0.5*z'Hz + ..
    if (flag==0)
        error('Unfeasible program');
    end
    x = A*x+B*zopt(2*N+1,1);
    control(i) = zopt(2*N+1,1);
 end
 
 % plots
 plot([X,O_lqr]);hold on;
 plot(traj(:,1),traj(:,2),'-sk','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.2,0.5,1]);
 l = legend('Feasible set','Terminal feasible set','Trajectory','Location','southwest');
 set(l,'FontSize',12);
 xlabel('$x_1$','interpreter','latex');
 ylabel('$x_2$','interpreter','latex');
 
 figure;
 stairs(control,'LineWidth',2,'Color',[0.9,0.2,0.3]);
 xlabel('Iteration');
 ylabel('Control');
 grid minor;
 legend('MPC Control law');
 
  
    