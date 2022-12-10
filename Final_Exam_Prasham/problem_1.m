addpath('utility')

%% Initialize
beta = 4/6;

% change body velocity
v_body_g = 0.01;

% change cycle time
t_cycle = 3;
l_stride = v_body_g*t_cycle;

% change number of cycles to simulate
num_of_cycles = 1;

%% Kinematic Phase
kinematic_phase(1) = 0;
kinematic_phase(2) = kinematic_phase(1) + 0.5;

for i = 3:6
    kinematic_phase(i) = kinematic_phase(i-2) + beta;
    if kinematic_phase(i) >= 1
        kinematic_phase(i) = kinematic_phase(i) - 1;
    end
end
kinematic_phase
%% foot trajectory

leg1 = [];
leg2 = [];
leg3 = [];
leg4 = [];
leg5 = [];
leg6 = [];

for leg = 1:6
    horizontal_pose = [];
    vertical_pose = [];
    horizontal_vel = [];
    vertical_vel = [];
    horizontal_pose_body = [];
    vertical_pose_body = [];
    horizontal_vel_body = [];
    vertical_vel_body = [];
    for t = 0:0.01:(t_cycle*num_of_cycles)
        [x, y, vx, vy] = final_exam_trajectory(t, l_stride, t_cycle, beta, kinematic_phase(leg), 0.05);
        if leg == 7
            horizontal_pose = [horizontal_pose, x - l_stride/2];
            horizontal_vel = [horizontal_vel, vx];
            horizontal_pose_body = [horizontal_pose_body, x - v_body_g*t - l_stride/2];
            horizontal_vel_body = [horizontal_vel_body, vx - v_body_g];
        
            vertical_pose = [vertical_pose, y];
            vertical_vel = [vertical_vel, vy];
            vertical_pose_body = [vertical_pose_body, y];
            vertical_vel_body = [vertical_vel_body, vy];
        else
            horizontal_pose = [horizontal_pose, x];
            horizontal_vel = [horizontal_vel, vx];
            horizontal_pose_body = [horizontal_pose_body, x - v_body_g*t];
            horizontal_vel_body = [horizontal_vel_body, vx - v_body_g];
        
            vertical_pose = [vertical_pose, y];
            vertical_vel = [vertical_vel, vy];
            vertical_pose_body = [vertical_pose_body, y];
            vertical_vel_body = [vertical_vel_body, vy];
        end
    end
    if leg == 1
        leg1 = [horizontal_pose', horizontal_vel', horizontal_pose_body', horizontal_vel_body', vertical_pose', vertical_vel', vertical_pose_body', vertical_vel_body'];
    elseif leg == 2
        leg2 = [horizontal_pose', horizontal_vel', horizontal_pose_body', horizontal_vel_body', vertical_pose', vertical_vel', vertical_pose_body', vertical_vel_body'];
    elseif leg == 3
        leg3 = [horizontal_pose', horizontal_vel', horizontal_pose_body', horizontal_vel_body', vertical_pose', vertical_vel', vertical_pose_body', vertical_vel_body'];
    elseif leg == 4
        leg4 = [horizontal_pose', horizontal_vel', horizontal_pose_body', horizontal_vel_body', vertical_pose', vertical_vel', vertical_pose_body', vertical_vel_body'];
    elseif leg == 5
        leg5 = [horizontal_pose', horizontal_vel', horizontal_pose_body', horizontal_vel_body', vertical_pose', vertical_vel', vertical_pose_body', vertical_vel_body'];
    elseif leg == 6
        leg6 = [horizontal_pose', horizontal_vel', horizontal_pose_body', horizontal_vel_body', vertical_pose', vertical_vel', vertical_pose_body', vertical_vel_body'];
    end
end

t = 0:0.01:(t_cycle*num_of_cycles);

%% Calculate Jacobian
theta1_arr = [];
theta2_arr = [];
theta3_arr = [];
theta4_arr = [];
theta5_arr = [];
theta6_arr = [];

theta1_dot_arr = [];
theta2_dot_arr = [];
theta3_dot_arr = [];
theta4_dot_arr = [];
theta5_dot_arr = [];
theta6_dot_arr = [];

robot_height = 0.08

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

% for leg 5
y5 = leg5(:, 7) - robot_height;
x5 = leg5(:, 3);
vx5 = leg5(:, 4);
vy5 = leg5(:, 8);

% for leg 6
y6 = leg6(:, 7) - robot_height;
x6 = leg6(:, 3);
vx6 = leg6(:, 4);
vy6 = leg6(:, 8);

T = [1, 300/5, (2*300)/5, (3*300)/5, (4*300)/5, 300];

for i = 1:length(t)
    x_body = v_body_g*t(i);

    [O01, O11, O21, O31, theta1, theta1_dot] = joint_kinematic(x1(i), y1(i), vx1(i), vy1(i));
    [O02, O12, O22, O32, theta2, theta2_dot] = joint_kinematic(x2(i), y2(i), vx2(i), vy2(i));    
    [O03, O13, O23, O33, theta3, theta3_dot] = joint_kinematic(x3(i), y3(i), vx3(i), vy3(i));
    [O04, O14, O24, O34, theta4, theta4_dot] = joint_kinematic(x4(i), y4(i), vx4(i), vy4(i));
    [O05, O15, O25, O35, theta5, theta5_dot] = joint_kinematic(x5(i), y5(i), vx5(i), vy5(i));
    [O06, O16, O26, O36, theta6, theta6_dot] = joint_kinematic(x6(i), y6(i), vx6(i), vy6(i));
      
    theta1_arr =[theta1_arr, theta1];
    theta2_arr =[theta2_arr, theta2];
    theta3_arr =[theta3_arr, theta3];
    theta4_arr =[theta4_arr, theta4];
    theta5_arr =[theta5_arr, theta5];
    theta6_arr =[theta6_arr, theta6];

    theta1_dot_arr =[theta1_dot_arr, theta1_dot];
    theta2_dot_arr =[theta2_dot_arr, theta2_dot];
    theta3_dot_arr =[theta3_dot_arr, theta3_dot];
    theta4_dot_arr =[theta4_dot_arr, theta4_dot];
    theta5_dot_arr =[theta5_dot_arr, theta5_dot];
    theta6_dot_arr =[theta6_dot_arr, theta6_dot];

    figure(1)
    title('robot leg')
    ld = 0.3;
    ld2 = 0.15;

    % robot base
    plot([x_body, x_body + ld],[0, 0],'linewidth',2)
    hold on

    % for leg 1
    plot([O01(2) + x_body + ld, O11(2)  + x_body + ld],[O01(3), O11(3)],'linewidth',2);
    hold on
    plot([O11(2) + x_body + ld, O21(2) + x_body + ld], [O11(3), O21(3)], 'linewidth', 2);
    hold on
    plot([O21(2) + x_body + ld, O31(2) + x_body + ld], [O21(3), O31(3)], 'linewidth', 2);
    hold on
    plot(O31(2) + x_body + ld, O31(3),'o','markers', 1)

    % for leg 2
    plot([O02(2) + x_body + ld, O12(2)  + x_body + ld],[O02(3),O12(3)],'linewidth',2);
    hold on
    plot([O12(2) + x_body + ld, O22(2) + x_body + ld], [O12(3), O22(3)], 'linewidth', 2);
    hold on
    plot([O22(2) + x_body + ld, O32(2) + x_body + ld], [O22(3), O32(3)], 'linewidth', 2);
    hold on
    plot(O32(2) + x_body + ld, O32(3),'o','markers', 1)

    % for leg 3
    plot([O03(2) + x_body + ld2, O13(2) + x_body + ld2],[O03(3),O13(3)],'linewidth',2);
    hold on
    plot([O13(2) + x_body + ld2, O23(2) + x_body + ld2], [O13(3), O23(3)], 'linewidth', 2);
    hold on
    plot([O23(2) + x_body + ld2, O33(2) + x_body + ld2], [O23(3), O33(3)], 'linewidth', 2);
    hold on
    plot(O33(2) + x_body + ld2, O33(3),'o','markers', 1)

    % for leg 4
    plot([O04(2) + x_body + ld2, O14(2)  + x_body + ld2],[O04(3),O14(3)],'linewidth',2);
    hold on
    plot([O14(2) + x_body + ld2, O24(2) + x_body + ld2], [O14(3), O24(3)], 'linewidth', 2);
    hold on
    plot([O24(2) + x_body + ld2, O34(2) + x_body + ld2], [O24(3), O34(3)], 'linewidth', 2);
    hold on
    plot(O34(2) + x_body + ld2, O34(3),'o','markers', 1)
    
    % for leg 5
    plot([O05(2) + x_body, O15(2)  + x_body],[O05(3),O15(3)],'linewidth',2);
    hold on
    plot([O15(2) + x_body, O25(2) + x_body], [O15(3), O25(3)], 'linewidth', 2);
    hold on
    plot([O25(2) + x_body, O35(2) + x_body], [O25(3), O35(3)], 'linewidth', 2);
    hold on
    plot(O35(2) + x_body, O35(3),'o','markers', 1)

    % for leg 6
    plot([O06(2) + x_body, O16(2)  + x_body],[O06(3),O16(3)],'linewidth',2);
    hold on
    plot([O16(2) + x_body, O26(2) + x_body], [O16(3), O26(3)], 'linewidth', 2);
    hold on
    plot([O26(2) + x_body, O36(2) + x_body], [O26(3), O36(3)], 'linewidth', 2);
    hold on
    plot(O36(2) + x_body, O36(3),'o','markers', 1)

    grid on

    hold off
    axis([-0.5,0.5,-0.2,0.2])
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
plot(t, theta1_arr(3, :));
title("theta leg 1")
legend("theta1", "theta2", "theta3")
saveas(figure(1),"theta leg 1","png")

figure(2)
plot(t, theta1_dot_arr(1, :));
hold on
plot(t, theta1_dot_arr(2, :));
grid on
plot(t, theta1_dot_arr(3, :));
title("theta dot leg 1")
legend("theta1 dot", "theta2 dot", "theta3 dot")
saveas(figure(2),"theta dot leg 1","png")

figure(3)
plot(t, theta2_arr(1, :));
hold on
plot(t, theta2_arr(2, :));
grid on
plot(t, theta2_arr(3, :));
title("theta leg 2")
legend("theta1", "theta2", "theta3")
saveas(figure(3),"theta leg 2","png")

figure(4)
plot(t, theta2_dot_arr(1, :));
hold on
plot(t, theta2_dot_arr(2, :));
grid on
plot(t, theta2_dot_arr(3, :));
title("theta dot leg 2")
legend("theta1 dot", "theta2 dot", "theta3 dot")
saveas(figure(4),"theta dot leg 2","png")

figure(5)
plot(t, theta3_arr(1, :));
hold on
plot(t, theta3_arr(2, :));
grid on
plot(t, theta3_arr(3, :));
title("theta leg 3")
legend("theta1", "theta2", "theta3")
saveas(figure(5),"theta leg 3","png")

figure(6)
plot(t, theta3_dot_arr(1, :));
hold on
plot(t, theta3_dot_arr(2, :));
grid on
plot(t, theta3_dot_arr(3, :));
title("theta dot leg 3")
legend("theta1 dot", "theta2 dot", "theta3 dot")
saveas(figure(6),"theta dot leg 3","png")

figure(7)
plot(t, theta4_arr(1, :));
hold on
plot(t, theta4_arr(2, :));
grid on
plot(t, theta4_arr(3, :));
title("theta leg 4")
legend("theta1", "theta2", "theta3")
saveas(figure(7),"theta leg 4","png")

figure(8)
plot(t, theta4_dot_arr(1, :));
hold on
plot(t, theta4_dot_arr(2, :));
grid on
plot(t, theta4_dot_arr(3, :));
title("theta dot leg 4")
legend("theta1 dot", "theta2 dot", "theta3 dot")
saveas(figure(8),"theta dot leg 4","png")

figure(9)
plot(t, theta5_arr(1, :));
hold on
plot(t, theta5_arr(2, :));
grid on
plot(t, theta5_arr(3, :));
title("theta leg 5")
legend("theta1", "theta2", "theta3")
saveas(figure(9),"theta leg 5","png")

figure(10)
plot(t, theta5_dot_arr(1, :));
hold on
plot(t, theta5_dot_arr(2, :));
grid on
plot(t, theta5_dot_arr(3, :));
title("theta dot leg 5")
legend("theta1 dot", "theta2 dot", "theta3 dot")
saveas(figure(10),"theta dot leg 5","png")

figure(11)
plot(t, theta6_arr(1, :));
hold on
plot(t, theta6_arr(2, :));
grid on
plot(t, theta6_arr(3, :));
title("theta leg 6")
legend("theta1", "theta2", "theta3")
saveas(figure(11),"theta leg 6","png")

figure(12)
plot(t, theta6_dot_arr(1, :));
hold on
plot(t, theta6_dot_arr(2, :));
grid on
plot(t, theta6_dot_arr(3, :));
title("theta dot leg 6")
legend("theta1 dot", "theta2 dot", "theta3 dot")
saveas(figure(12),"theta dot leg 6","png")