%% 
clc;
clear all;
close all;

%% problem definition
n = 2;                  % state size
A = [4/3 -2/3;  1 0];   % state transition matrix
B = [1;0];              % control transition matrix
C = [-2/3 1];           % output mapping matrix
x0 = [10;10];           % initial state


%% EXO 1
N = 8;                % horizon
Q = C'*C + 0.001*eye(n); % output weighting matrix
R = 0.001;               % control weighting matrix

% dynamic programming solving for lqr
K_lqr = lqr_ricatti_solver(A,B,Q,R,N);  % feedback laws for lqr optimal control over horizon of size N
K = K_lqr(1,:);                         % first step feedback law



%% EXO 2
max_simu_step = 40;                 % simulation steps
u_stored = zeros(max_simu_step,1);  % control container
y_stored = zeros(max_simu_step,1);  % output container
x_stored = zeros(max_simu_step,2);  % state container
y_pred = zeros(3*N,1);              % predicted output container
x_pred = zeros(3*N,2);              % prediced state container
pred_points = [1,3,8];            % for plotting purposes
count = 1;                              
x = x0;

for i=1:max_simu_step
    if sum(i==pred_points)
        y_pred(count:count+N,:) = predict_on_horizon(A,B,C,K_lqr,N,x);
        count = count+N;
    end
    x_stored(i,:) = x;
    
    % output and control, closed loop
    y = C*x;
    y_stored(i) = y;
    u = K*x;
    u_stored(i) = u;

    % update 
    x = A*x + B*u;
end

%plots
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(1,2,1); stairs(y_stored,'b:d','LineWidth',1.5,'MarkerSize',5); hold on; stairs(u_stored,'r:d','LineWidth',1.5,'MarkerSize',5);
legend('Output sequence','Control sequence');
title(strcat('Output and control sequence for the closed loop with LQR control (N=',num2str(N),')'));

%figure;
subplot(1,2,2); plot(y_stored(:,1),'b-o','LineWidth',1.5);
hold on;
plot((pred_points(1):(pred_points(1)+N-1)),y_pred(1:N,1),'-.','Color',[0.8,0.8,0.8],'LineWidth',3)
plot((pred_points(2):pred_points(2)+N-1),y_pred(N+1:2*N,1),'-.','Color',[0.6,0.6,0.6],'LineWidth',3)
plot((pred_points(3):pred_points(3)+N-1),y_pred(2*N+1:3*N,1),'-.','Color',[0.4,0.4,0.4],'LineWidth',3)

title(strcat('Output trajectory and predicted trajectories for the closed loop with LQR control (N=',num2str(N),')'));


%% EXO 3

