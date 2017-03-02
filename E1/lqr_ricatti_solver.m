function K = lqr_ricatti_solver(A,B,Q,R,N)
%% HEADER

H = Q;
for i=1:N
   K = -inv((R+B'*H*B))*B'*H*A;
   H = Q+K'*R*K +(A+B*K)'*H*(A+B*K);
end

end
