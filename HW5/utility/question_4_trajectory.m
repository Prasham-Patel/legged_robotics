function [x, y, vx, vy] = question_4_trajectory(t, l_stride, t_cycle, beta, kinematic_phase, lambda)
    vy_change = 0.2;
    y2_start = 0.05;
    
    cycle_num = floor(t/t_cycle);

    phase1 = beta+kinematic_phase;
    phase2 = kinematic_phase;
    if phase1 >= 1
        phase1 = phase1-1;
    end
    if phase2 == 0
        phase2 = 1;
    end

    phase1 = phase1 + cycle_num;
    phase2 = phase2 + cycle_num;

    l_swing = l_stride;
    t1 = t_cycle*phase1;
    t2 = t_cycle*phase1 + t_cycle*lambda;
    t3 = t_cycle*phase2 - t_cycle*lambda;
    t4 = t_cycle*phase2;

    if t < t1
        vx = 0;
        vy = 0;
        x = 0;
        y = 0;
    end

    if t >= t1 && t < t2
        x = 0;
        vx = 0;
        y_co_eff = get_mini_acc_trajectory_co_eff(0, y2_start, 0, vy_change, t1, t2);
        [y, vy] = at_time(t, y_co_eff);
    end

    if t >= t2 && t < t3
        x_co_eff = get_mini_acc_trajectory_co_eff(0, l_swing, 0, 0, t2, t3);
        y_co_eff = get_mini_acc_trajectory_co_eff(y2_start, y2_start, vy_change, -vy_change, t2, t3);
        [x, vx] = at_time(t, x_co_eff);
        [y, vy] = at_time(t, y_co_eff);
    end

    if t>= t3 && t <= t4
        x = l_swing;
        vx = 0;
        y_co_eff = get_mini_acc_trajectory_co_eff(y2_start, 0, -vy_change, 0, t3, t4);
        [y, vy] = at_time(t, y_co_eff);
    end

    if t > t4
        x = l_swing;
        vx = 0;
        y = 0;
        vy = 0;
    end
    
    x = x + l_stride*cycle_num;

end