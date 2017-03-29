function traj = simulate_traj(dynamic, start, max_iter, constraint)
% \brief :  Simulate the trajectory driven by the dynamic matrix starting at
%           start point, until max iter is reached or the running points stops
%           complying with the constraints   

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