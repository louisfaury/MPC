 %% MPC - Group 2 
 %  Script for EXERCICE 1 
 clc;
 clear all;
 close all;
 
 % some definitions 
 alpha = pi/6;
 beta = 0.8; 
 A = [  
        cos(alpha) , sin(alpha) ; 
        -sin(alpha), cos(alpha)
     ]*beta;
 H = [ 
        cos(pi/3)  , sin(pi/3)  ;
        -cos(pi/3) , -sin(pi/3) ;
        sin(pi/3)  , -cos(pi/3) ;
        -sin(pi/3) , cos(pi/3)  ;
     ];
 h = [2 1 2 5]';
 
 % State space 
 X = Polyhedron(H,h);
 
 %% Task 1 : compute the largest invariant set of the constraint system
 convergence = false;   % convergence flag
 O = X;                 % current estimate for O_infinity

 % computing largest invariant set
 while (~convergence)
     Op = Polyhedron(O.A*A,O.b);                    % current Omega pre-set
     On = Polyhedron( [Op.A;O.A] , [Op.b;O.b] );    % update rule
     convergence = (O == On);                       % convergence check
     O = On;                                        % update 
 end
 O.plot;
 
 plot([X,O]); hold on;
 l = legend('$\bf{X}$','$\mathcal{O}_\infty$');
 set(l,'Interpreter','latex');
 
 %% Task 2 : trajectory plots
 
 x_out = [-3;2.2];
 x_in = {[-3;1.5],[-2;1],[-1;1.5],[0; 1.2], [1.4; 0], [2; 1], [-1; 0]};
 
 traj_out = simulate_traj(A, x_out, 20, X);
 plot(traj_out(1,:),traj_out(2,:),'bo-','MarkerSize',8,'MarkerEdgeColor',[0.1,0.1,0.1],'MarkerFaceColor',[0.1,0.1,0.1]);
 plot(x_out(1),x_out(2),'s','MarkerSize',10,'MarkerEdgeColor',[0.1,0.1,0.1],'MarkerFaceColor',[0.99,0.99,0.99]);

 for x=cell2mat(x_in)
    traj_in = simulate_traj(A, x, 20, X);
    plot(traj_in(1,:),traj_in(2,:),'bo-','MarkerSize',5,'MarkerEdgeColor',[0.1,0.1,0.1],'MarkerFaceColor',[0.1,0.1,0.1]);
    plot(x(1),x(2),'s','MarkerSize',6,'MarkerEdgeColor',[0.1,0.1,0.1],'MarkerFaceColor',[0.99,0.99,0.99]);
 end
  
 