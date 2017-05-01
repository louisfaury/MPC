%% MPC - Exercice Session 5 
%  Group 7

%%
clc; 
clear all;
close all;

%% Exercice 1 
% variable defs 
A = [ 0.7115 -0.4345; 
      0.4345 0.8853];
B = [0.2173 0.0573]';
C = [0 1];

% augmented state 
f  = zeros(3,1);     % f_k = [x_k;d_k]
Bd = zeros(size(B));
Af = [ A Bd; zeros(1,size(A,2)) 1];
Bf = [B;0];
Cd = 1;
Cf = [C Cd];

% place error's dynamic's poles 
F = [0.5; 0.6; 0.7];
L = (place(Af',-Cf',F))';

% % simulate the system and the estimate's dynamics 
% max_iter    = 30;
% x           = [1;2].*ones(2,max_iter);          % state
% d           = 0.2*ones(1,max_iter);             % disturbance
% u           = 0;                                % constant zero control 
% f           = [3; 0; 0].*ones(3,max_iter); % augmented state : [estimated state ; estimated disturbance]
% y           = (C*x + d).*ones(1,max_iter);      % output 
% for iter=1:max_iter
%    f(:,iter+1) = Af*f(:,iter) + [B;0]*u + L*(Cf*f(:,iter)-y(:,iter));
%    x(:,iter+1) = A*x(:,iter) + B*u;
%    y(:,iter+1) = C*x(:,iter) + d(:,iter);
% end
% 
% % plot
% plot(x(1,:),x(2,:),'r','LineWidth',2); hold on,
% plot(f(1,:),f(2,:),'b','LineWidth',2);
% grid minor;
% h = legend('True state $x$','Estimated State $\hat{x}$');
% set(h,'interpreter','latex','FontSize',14)
% xlabel('$x_1$','interpreter','latex','FontSize',14);
% ylabel('$x_2$','interpreter','latex','FontSize',14);
% figure;
% stairs(d,'r','LineWidth',2); hold on;
% stairs(f(3,:),'b','LineWidth',2);
% h = legend('$d$','$\hat{d}$');
% set(h,'interpreter','latex','FontSize',14);
% xlabel('Time');
% grid minor;

%% Exercice 2 
% Minimizing u^2 and respecting constraints (YALMIP implementation)
% Demo implementation // do not run 
% r = 0.5;            % Target r = 0.5
% g = sdpvar(3,1);
% u = [ 0 0 1]*g;
% Constraints = [-3 <= u <= 3,[eye(2)-A, -B; C 0]*g==[0;0;r]];    % defining constraints on u, and minimizing u^2
% optimize(Constraints,g);
% g = value(g);                                                   % [xs_1,xs_2,u_s]
% xs = g(1:2,1);
% us = g(3,1);

%% Exercice 3
Q = ones(2);
R = 1;
N = 5;
P = dlyap(A,Q);
Hu = [ 1; -1]; hu = [3;3];
r  = 1;
% simulate the system and the estimate's dynamics 
max_iter    = 50;
x           = [1;2].*ones(2,max_iter);          % state
d           = 0.2*ones(1,max_iter);             % disturbance
f           = [3; 0; 0].*ones(3,max_iter);      % augmented state : [estimated state ; estimated disturbance]
y           = (C*x + d).*ones(1,max_iter);      % output
u           = zeros(1,max_iter);
[xs,us]     = compute_steady_state_est(r,A,B,C,Cd,f(3,1));
dx          = x-xs.*ones(size(x));
for iter=1:max_iter
   uMPC = us + mpc_yalmip_controller(N,A,B,Q,P,R,f(1:2,iter),Hu,hu,us);
   f(:,iter+1) = Af*f(:,iter) + [B;0]*uMPC(1) + L*(Cf*f(:,iter)-y(:,iter));
   x(:,iter+1) = A*x(:,iter) + B*uMPC(1);
   y(:,iter+1) = C*x(:,iter) + d(:,iter);
   u(1,iter) = uMPC(1);
   [xs,us] = compute_steady_state_est(r,A,B,C,Cd,f(3,iter+1));
end


% plot
%state
plot(x(1,:),x(2,:),'r','LineWidth',2); hold on,
plot(f(1,:),f(2,:),'b','LineWidth',2);
grid minor;
h = legend('True state $x$','Estimated State $\hat{x}$');
set(h,'interpreter','latex','FontSize',14)
xlabel('$x_1$','interpreter','latex','FontSize',14);
ylabel('$x_2$','interpreter','latex','FontSize',14);
%disturbance
figure;
stairs(d,'r','LineWidth',2); hold on;
stairs(f(3,:),'b','LineWidth',2);
h = legend('$d$','$\hat{d}$');
set(h,'interpreter','latex','FontSize',14);
xlabel('Time');
grid minor;
% input
figure; hold on;
a = stairs(u,'k','LineWidth',2);
b = plot([1,iter],[3,3],'r','LineWidth',2);
plot([1,iter],[-3,-3],'r','LineWidth',2);
legend([a,b],{'Input u','Input constraints'});
% output 
figure; hold on;
a = plot(y,'k','LineWidth',2);
b = plot([1,iter],[r,r],'r','LineWidth',2);
legend([a,b],{'Output y','Output reference'});





