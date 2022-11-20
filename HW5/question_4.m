addpath('utility')

%% Initialize
beta = 0.75;

% change body velocity
v_body_g = 0.1;

% change cycle time
t_cycle = 1.5;
l_stride = v_body_g*t_cycle;

% change number of cycles to simulate
num_of_cycles = 1;

%% Kinematic Phase
kinematic_phase(1) = 0;
kinematic_phase(2) = kinematic_phase(1) + 0.5;

for i = 3:4
    kinematic_phase(i) = kinematic_phase(i-2) + beta;
    if kinematic_phase(i) >= 1
        kinematic_phase(i) = kinematic_phase(i) - 1;
    end
end

%% foot trajectory

leg1 = [];
leg2 = [];
leg3 = [];
leg4 = [];

for leg = 1:4
    horizontal_pose = [];
    vertical_pose = [];
    horizontal_vel = [];
    vertical_vel = [];
    horizontal_pose_body = [];
    vertical_pose_body = [];
    horizontal_vel_body = [];
    vertical_vel_body = [];
    for t = 0:0.01:(t_cycle*num_of_cycles)
        [x, y, vx, vy] = question_4_trajectory(t, l_stride, t_cycle, beta, kinematic_phase(leg), 0.05);
        horizontal_pose = [horizontal_pose, x];
        horizontal_vel = [horizontal_vel, vx];
        horizontal_pose_body = [horizontal_pose_body, x - v_body_g*t];
        horizontal_vel_body = [horizontal_vel_body, vx - v_body_g];
    
        vertical_pose = [vertical_pose, y];
        vertical_vel = [vertical_vel, vy];
        vertical_pose_body = [vertical_pose_body, y];
        vertical_vel_body = [vertical_vel_body, vy];
    end
    if leg == 1
        leg1 = [horizontal_pose', horizontal_vel', horizontal_pose_body', horizontal_vel_body', vertical_pose', vertical_vel', vertical_pose_body', vertical_vel_body'];
    elseif leg == 2
        leg2 = [horizontal_pose', horizontal_vel', horizontal_pose_body', horizontal_vel_body', vertical_pose', vertical_vel', vertical_pose_body', vertical_vel_body'];
    elseif leg == 3
        leg3 = [horizontal_pose', horizontal_vel', horizontal_pose_body', horizontal_vel_body', vertical_pose', vertical_vel', vertical_pose_body', vertical_vel_body'];
    elseif leg == 4
        leg4 = [horizontal_pose', horizontal_vel', horizontal_pose_body', horizontal_vel_body', vertical_pose', vertical_vel', vertical_pose_body', vertical_vel_body'];
    end
end

t = 0:0.01:(t_cycle*num_of_cycles);

%% Calculate Jacobian
theta1_arr = [];
theta2_arr = [];
theta3_arr = [];
theta4_arr = [];

theta1_dot_arr = [];
theta2_dot_arr = [];
theta3_dot_arr = [];
theta4_dot_arr = [];

% Robot parameter
% if changing leg dimensions also change in joint_kinematics function
ld = 0.5;
l1 = 0.3;
l2 = 0.3;
robot_height = 0.4;

for i = 1:length(t)
    x_body = v_body_g*t(i);
    
    % for leg 1
    y1 = leg1(:, 7) - robot_height;
    x1 = leg1(:, 3);
    vx1 = leg1(:, 4);
    vy1 = leg1(:, 8);

    % for leg 2
    y2 = leg2(:, 7) - robot_height;
    x2 = leg2(:, 3);
    vx2 = leg2(:, 4);
    vy2 = leg2(:, 8);

    % for leg 3
    y3 = leg3(:, 7) - robot_height;
    x3 = leg3(:, 3);
    vx3 = leg3(:, 4);
    vy3 = leg3(:, 8);

    % for leg 4
    y4 = leg4(:, 7) - robot_height;
    x4 = leg4(:, 3);
    vx4 = leg4(:, 4);
    vy4 = leg4(:, 8);

    [O01, O11, O21, theta1, theta1_dot] = joint_kinematic(x1(i), y1(i), vx1(i), vy1(i));
    [O02, O12, O22, theta2, theta2_dot] = joint_kinematic(x2(i), y2(i), vx2(i), vy2(i));    
    [O03, O13, O23, theta3, theta3_dot] = joint_kinematic(x3(i), y3(i), vx3(i), vy3(i));
    [O04, O14, O24, theta4, theta4_dot] = joint_kinematic(x4(i), y4(i), vx4(i), vy4(i));
      
    theta1_arr =[theta1_arr, theta1];
    theta2_arr =[theta2_arr, theta2];
    theta3_arr =[theta3_arr, theta3];
    theta4_arr =[theta4_arr, theta4];  

    theta1_dot_arr =[theta1_dot_arr, theta1_dot];
    theta2_dot_arr =[theta2_dot_arr, theta2_dot];
    theta3_dot_arr =[theta3_dot_arr, theta3_dot];
    theta4_dot_arr =[theta4_dot_arr, theta4_dot];

    figure(1)
    title('robot leg')

    % robot base
    plot([x_body, x_body + ld],[0, 0],'linewidth',2)
    hold on

    % for leg 1
    plot([O01(1) + x_body + ld, O11(1)  + x_body + ld],[O01(2),O11(2)],'linewidth',2);
    hold on
    plot([O11(1) + x_body + ld, O21(1) + x_body + ld], [O11(2), O21(2)], 'linewidth', 2);
    hold on
    plot(O21(1) + x_body + ld, O21(2),'o','markers',20)

    % for leg 2
    plot([O02(1) + x_body + ld, O12(1)  + x_body + ld],[O02(2),O12(2)],'linewidth',2);
    hold on
    plot([O12(1) + x_body + ld, O22(1) + x_body + ld], [O12(2), O22(2)], 'linewidth', 2);
    hold on
    plot(O22(1) + x_body + ld, O22(2),'o','markers',20)

    % for leg 3
    plot([O03(1) + x_body, O13(1) + x_body],[O03(2),O13(2)],'linewidth',2);
    hold on
    plot([O13(1) + x_body, O23(1) + x_body], [O13(2), O23(2)], 'linewidth', 2);
    hold on
    plot(O23(1) + x_body, O23(2),'o','markers',20)

    % for leg 4
    plot([O04(1) + x_body, O14(1)  + x_body],[O04(2),O14(2)],'linewidth',2);
    hold on
    plot([O14(1) + x_body, O24(1) + x_body], [O14(2), O24(2)], 'linewidth', 2);
    hold on
    plot(O24(1) + x_body, O24(2),'o','markers',20)
    grid on
    hold off
    axis([-1.5,1.5,-1.2,1.2])
    M(i) = getframe(gcf);
end
movie(M)
videofile = VideoWriter('leg_swing.avi','Uncompressed AVI');
open(videofile)
writeVideo(videofile,M)
close(videofile)

figure(1)
plot(t, theta1_arr(1, :));
hold on
plot(t, theta1_arr(2, :));
grid on
title("theta leg 1")
saveas(figure(1),"theta leg 1","png")

figure(2)
plot(t, theta1_dot_arr(1, :));
hold on
plot(t, theta1_dot_arr(2, :));
grid on
title("theta dot leg 1")
saveas(figure(2),"theta dot leg 1","png")

figure(3)
plot(t, theta2_arr(1, :));
hold on
plot(t, theta2_arr(2, :));
grid on
title("theta leg 2")
saveas(figure(3),"theta leg 2","png")

figure(4)
plot(t, theta2_dot_arr(1, :));
hold on
plot(t, theta2_dot_arr(2, :));
grid on
title("theta dot leg 2")
saveas(figure(4),"theta dot leg 2","png")

figure(5)
plot(t, theta3_arr(1, :));
hold on
plot(t, theta3_arr(2, :));
grid on
title("theta leg 3")
saveas(figure(5),"theta leg 3","png")

figure(6)
plot(t, theta3_dot_arr(1, :));
hold on
plot(t, theta3_dot_arr(2, :));
grid on
title("theta dot leg 3")
saveas(figure(6),"theta dot leg 3","png")

figure(7)
plot(t, theta4_arr(1, :));
hold on
plot(t, theta4_arr(2, :));
grid on
title("theta leg 4")
saveas(figure(7),"theta leg 4","png")

figure(8)
plot(t, theta4_dot_arr(1, :));
hold on
plot(t, theta4_dot_arr(2, :));
grid on
title("theta dot leg 4")
saveas(figure(8),"theta dot leg 4","png")