function K_on_horizon = lqr_ricatti_solver(A,B,Q,R,N)
% @brief DP solving of the MPC
% @param   x_{k+1} = Ax_k + Bu_k
%          Q,R are the weighting matrixes of the MPC problem
%          N is the horizon
% @returns The sequence of linear feedback laws for 1,..,N

K_on_horizon = zeros(N,2);

H = Q;
for i=N:-1:1
    K = -inv((R+B'*H*B))*B'*H*A;
    H = Q+K'*R*K +(A+B*K)'*H*(A+B*K);
    K_on_horizon(i,:) = K;
end

end
