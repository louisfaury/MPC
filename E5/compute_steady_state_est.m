function [xs,us] = compute_steady_state_est(r,A,B,C,Cd,d_hat)
    g = sdpvar(3,1);
    u = [ 0 0 1]*g;
    Constraints = [-3 <= u <= 3,[eye(2)-A, -B; C 0]*g==[0;0;r-Cd*d_hat]];    % defining constraints on u, and minimizing u^2
    optimize(Constraints,g);
    g = value(g);                                                           % [xs_1,xs_2,u_s]
    xs = g(1:2,1);
    us = g(3,1);
end