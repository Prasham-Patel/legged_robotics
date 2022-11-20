function [O0, O1, O2, theta, theta_dot] = joint_kinematic(x, y, vx, vy)
    l1 = 0.3;
    l2 = 0.3;

    l = (x^2 + (y)^2)^0.5;
    d = (l1^2 + l2^2 - l^2)/(2*l1*l2);
    theta2 = pi - atan2((1-d^2)^0.5, d);
    d2 = (l1^2 + l^2 - l2^2)/(2*l1*l);
    theta1 = -atan2((1 - d2^2)^0.5, d2) + atan2(y, x);
    
    A1 = h_transform(theta1, 0, l1, 0);
    A2 = h_transform(theta2, 0, l2, 0);
    A3 = A1*A2;
    
    z0 = [0, 0, 1]';
    z1 = [0, 0, 1]';
    O0 = [0, 0, 0]';
    O1 = A1(1:3, 4);
    O2 = A3(1:3, 4);
    
    jacobian = [cross(z0, (O2 - O0)), cross(z1, (O2 - O1));
                z0, z1];
    j_inv = pinv(jacobian);
    theta_dot = j_inv(:, 1:2)*[vx, vy]';
    theta = [theta1; theta2];
end