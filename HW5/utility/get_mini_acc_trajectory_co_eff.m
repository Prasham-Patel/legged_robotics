function co_efficient = get_mini_acc_trajectory_co_eff(xi, xf, vi, vf, ti, tf)
    time_matrix = [1, ti, ti^2, ti^3;
                   0, 1, 2*ti, 3*ti^2;
                   1, tf, tf^2, tf^3;
                   0, 1, 2*tf, 3*tf^2];
    co_efficient = pinv(time_matrix)* [xi, vi, xf, vf]';
end