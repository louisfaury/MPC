function [H,h,G,g,T,t] = mpc_precompute(N,A,B,Q,P,R,x0,Hx,hx,Hu,hu,Xf)
%! computes the different matrix needed for solving the MPC quadratic program !%

% global cost matrix H
 H = zeros(3*N,3*N);
 h = zeros(3*N,1);
 H(1:2*(N-1),1:2*(N-1)) = kron(eye(N-1),Q);
 H(2*N-1:2*N,2*N-1:2*N) = P;
 H(2*N+1:3*N,2*N+1:3*N) = kron(eye(N),R);
 
 % global dynamic matrix
 t = zeros(2*N,1);
 t(1:2,1) = -A*x0;
 T = zeros(2*N,3*N);
 T(1:2*N,1:2*N) = -kron(eye(N),eye(2));
 T(1:2*N,1:2*N) = T(1:2*N,1:2*N) + kron(diag(-ones(N-1,1),-1),A);
 T(1:2*N,2*N+1:3*N) = kron(eye(N),B);
 
 % global contraint matrix 
 G = zeros(6*N-4+size(Xf.A,1),3*N);
 g = zeros(6*N-4+size(Xf.A,1),1);
 G(1:4*(N-1),1:2*(N-1)) = kron(eye(N-1),Hx);
 g(1:4*(N-1)) = repmat(hx,N-1,1);
 G(4*N-3:4*N-3+size(Xf.A,1)-1,2*N-1:2*N) = Xf.A;
 g(4*N-3:4*N-3+size(Xf.A,1)-1) = Xf.b;
 G(4*N-3+size(Xf.A,1):6*N-4+size(Xf.A,1),2*N+1:3*N) =  kron(eye(N),Hu);
 g(4*N-3+size(Xf.A,1):6*N-4+size(Xf.A,1)) = repmat(hu,N,1);
 
end