function x_pred = predict_on_horizon(A,B,K,N,x_0)
% @brief Make predictions on horizon using 'open loop' control from the LQR solution (no receding horizon)
% @param   x_pred : container for predictions
%          x_{k+1} = Ax_k + Bu
%          K : optimal control law given by LQR solution
%          N : prediction horizon
%          x_0 : starting point
% @returns The sequence of predicted states for i=1..n from x0

x = x_0;
x_pred = zeros(N+1,2);
x_pred(1,:) = x;

for i=1:N
    x = A*x + B*K(i,:)*x;
    x_pred(i+1,:) = x;
end

end