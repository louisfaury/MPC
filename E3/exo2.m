%% MPC - Group 2 
 %  Script for EXERCICE 2 
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
 B = [0.5 0.5]';
 H = [ 
        cos(pi/3)  , sin(pi/3)  ;
        -cos(pi/3) , -sin(pi/3) ;
        sin(pi/3)  , -cos(pi/3) ;
        -sin(pi/3) , cos(pi/3)  ;
     ];
 h = [2 1 2 5]';
 G = [-1 1]'; 
 g = 0.5*[1 1]';
 
 % State space 
 X = Polyhedron(H,h);
 
  %% Task 1 : compute the largest control invariant set of the constraint system
 convergence = false;   % convergence flag
 Oc = X;                 % current estimate for O_infinity

 %! ------------------------------------------------------ !%
 %      computing largest control invariant set             %
 %! ------------------------------------------------------ !%
 while (~convergence)
     Op = projection(Polyhedron([ Oc.A*A,Oc.A*B ; zeros(2,2),G ], [Oc.b;g]),(1:2) );    % current Omega pre-set
     On = Polyhedron( [Op.A;Oc.A] , [Op.b;Oc.b] );                                      % update rule
     convergence = (Oc == On);                                                          % convergence check
     Oc = On;                                                                           % update 
 end
 %! ------------------------------------------------------ !%
 
 plot([X,Oc]); hold on;
 l = legend('$\bf{X}$','$\mathcal{C}_\infty$');
 set(l,'Interpreter','latex','FontSize',14);
 
 
 %% Task 2 : compute the optimal LQR controller and computes its maximum invariant set 
 
 Q = eye(2);
 R = 1;
 K = -dlqr(A,B,Q,R); % optimal LQR gain 
 
 % Defines new constraint set 
 X_lqr = Polyhedron([X.A;G*K],[X.b;g]);
 convergence = false;   % convergence flag
 O_lqr = X_lqr;         % current estimate for O_infinity

 %! ------------------------------------------------------ !%
 %      computing largest invariant set for lqr             %
 %! ------------------------------------------------------ !%
 while (~convergence)
     Op = Polyhedron(O_lqr.A*A,O_lqr.b);                    % current Omega pre-set
     On = Polyhedron( [Op.A;O_lqr.A] , [Op.b;O_lqr.b] );    % update rule
     convergence = (O_lqr == On);                           % convergence check
     O_lqr = On;                                            % update 
 end
  %! ------------------------------------------------------ !%

  
 figure;
 plot([X,Oc,X_lqr,O_lqr]); hold on;
 l = legend('$\bf{X}$','$\mathcal{C}_\infty$','$\bf{X}^{lqr}$','$\mathcal{O}_\infty^{lqr}$');
 set(l,'Interpreter','latex','FontSize',14);
 