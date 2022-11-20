addpath('utility')

%% Initialize
v_leg_g = 0.16;
v_body_g = 0.04;
l_stride = 0.16;
max_height = 0.03; % assumed max_height

% Calculate Beta
beta = (v_leg_g-v_body_g)/v_leg_g;

% Calculate Cycle time period
t_cycle = l_stride/v_body_g;

% Calculate Transfer time
t_transfer = t_cycle*(1 - beta);

% calculate Stance time
t_stance = t_cycle*beta;

%swing length
l_swing = l_stride;

%% Calculate Trajecrtory

horizontal_pose = [];
vertical_pose = [];
horizontal_vel = [];
vertical_vel = [];
horizontal_pose_body = [];
vertical_pose_body = [];

co_eff = get_mini_acc_trajectory_co_eff(0, l_swing, 0, 0, 0, t_transfer);
for t = 0:0.01:t_transfer
    [x, v] = at_time(t, co_eff);
    
    horizontal_pose = [horizontal_pose, x];
    horizontal_vel = [horizontal_vel, v];
    horizontal_pose_body = [horizontal_pose_body, x-v_body_g*t];

    vertical_pose = [vertical_pose, max_height*sin((x/l_swing)*pi)];
    vertical_vel = [vertical_vel, max_height*(pi/l_swing)*cos((x/l_swing)*pi)];
    vertical_pose_body = [vertical_pose_body, max_height*sin((x/l_swing)*pi)];
end

t = 0:0.01:t_transfer;

figure(1)
plot(t, horizontal_pose)
grid on
title('horizontal pose')
saveas(figure(1), "horizontal_pose", "png")

figure(2)
plot(t, horizontal_vel)
grid on
title('horizontal vel')
saveas(figure(2), "horizontal_vel", "png")

figure(3)
plot(t, vertical_pose)
grid on
title('vertical pose')
saveas(figure(3), "vertical_pose", "png")

figure(4)
plot(t, vertical_vel)
grid on
title('horizontal vel')
saveas(figure(4), "vertical_vel", "png")

figure(5)
plot(horizontal_pose, vertical_pose)
grid on
title('leg trajectory ground frame')
saveas(figure(5), "leg trajectory ground frame", "png")

figure(6)
plot(horizontal_pose_body, vertical_pose_body)
grid on
title('leg trajectory body frame')
saveas(figure(6), "leg trajectory body frame", "png")






