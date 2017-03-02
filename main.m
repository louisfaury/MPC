%% 
clc;
clear all;
close all;

%% problem definition
n = 2; %state size
A = [4/3 -2/3;  1 0];
B = [1;0];
C = [-2/3 1];
x0 = [10;10];

%% EXO 1
N = 20; % horizon 
Q = C'*C + 0.001*eye(n);
R = 0.001;

% dynamic programming solving for lqr 
K_lqr = lqr_ricatti_solver(A,B,Q,R,N); % gain for lqr optimal control 


%% EXO 2
max_simu = 30;
x = x0;
y = C*x;

for i=1:max_simu
    u = K_lqr*x;
    x = A*x + B*u;
    y = C*x;
    store(i,:) = x;
end
