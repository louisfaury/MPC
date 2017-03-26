function traj = simulate_traj(dynamic, start, max_iter, constraint)

iter = 1;
x = start;
full_traj = zeros(2,max_iter);
full_traj(:,iter) = x;

while ( iter<max_iter )
    iter = iter+1;
    x = dynamic*x;
    full_traj(:,iter) = x;
    
    if (~constraint.contains(x))
        break;
    end
end

traj = full_traj(:,1:iter);