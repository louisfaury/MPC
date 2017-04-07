%% MPC class
%% E4 script for exercice 2 

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

 
 % Computing LQR controller and LQR terminal with MPT3
 sys = LTISystem('A',A,'B',B);
 sys.x.max = [5; 0.2]; sys.x.min = [-5; -0.2];
 sys.u.max = 1.75; sys.u.min = -1.75;
 sys.x.penalty = QuadFunction(Q);
 sys.u.penalty = QuadFunction(R);
 mpt_res.weight = sys.LQRPenalty.weight;
 mpt_res.set = sys.LQRSet;
 mpt_res.controller = sys.LQRGain;
 
 %%MPC algorithm using YALMIP    
 x = [3;0];
 max_iter = 30;
 traj = zeros(max_iter,2);
 control = zeros(max_iter,1);
 for i=1:20
     traj(i,:) = x;
     
     uopt = mpc_yalmip_controller(N,A,B,Q,mpt_res.weight,R,x,Hx,hx,Hu,hu, mpt_res.set);

    x = A*x+B*uopt(1);
    control(i) = uopt(1);
 end
 
 % plots
 plot([X, mpt_res.set]);hold on;
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
 
  
    