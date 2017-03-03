function [K_on_horizon, H0] = lqr_ricatti_solver(A,B,Q,R,N)
% @brief DP solving of the MPC
% @param   x_{k+1} = Ax_k + Bu_k
%          Q,R are the weighting matrixes of the MPC problem
%          N is the horizon
% @returns K_on_horizon, the sequence of linear feedback laws for 1,..,N
%          H0, such that cost is x_k'H0x_k           

K_on_horizon = zeros(N,2);
H_on_horizon = zeros(N,2);

H = Q;
for i=N:-1:1
    K = -inv((R+B'*H*B))*B'*H*A;
    H = Q+K'*R*K +(A+B*K)'*H*(A+B*K);
    K_on_horizon(i,:) = K;
end

H0=H;

end
