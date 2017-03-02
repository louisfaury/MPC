function y_pred = predict_on_horizon(A,B,C,K,N,x_0)
% @brief Make predictions on horizon using 'open loop' control from the LQR solution (no receding horizon)
% @param   x_pred : container for predictions
%          x_{k+1} = Ax_k + Bu
%          K : optimal control law given by LQR solution
%          N : prediction horizon
%          x_0 : starting point
% @returns The sequence of predicted states for i=1..n from x0

x = x_0;
y_pred = zeros(N+1,1);
y_pred(1,:) = C*x;

for i=1:N
    x = A*x + B*K(i,:)*x;
    y_pred(i+1,:) = C*x;
end

end