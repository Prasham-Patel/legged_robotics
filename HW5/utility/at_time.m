function [x, v] = at_time(t, co_eff)
    time_matrix = [1, t, t^2, t^3;
                   0, 1, 2*t, 3*t^2];
    traj = (time_matrix*co_eff)';
    x = traj(1);
    v = traj(2);
end