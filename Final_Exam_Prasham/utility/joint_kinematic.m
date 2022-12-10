function [O0, O1, O2, O3, theta, theta_dot] = joint_kinematic(y, z, vy, vz)

    l1 = 0;
    l2 = 0.050;
    l3 = 0.100;

    x = 0.050;
    vx = 0;

    theta1 = atan2(y, x);

    H = sqrt(x^2 + y^2 + z^2);
    D = (l2^2 +l3^2 - H^2)/(2*l2*l3);
    E = (l2^2 + H^2 - l3^2)/(2*l2*H);

    theta3 = atan2(sqrt(1-D^2), D) - (pi/2);

    phi = atan2(sqrt(H^2 - z^2), -z);
%     rad2deg(phi)
%         atan2(sqrt(1-E^2), E)
    theta2 = (phi) + atan2(sqrt(1-E^2), E) - pi/2;
    
    % theta, alpha, a, d
    A1 = h_transform(theta1, -pi/2, 0, 0);
    A2 = h_transform(-theta2, 0, l2, 0);
    A3 = h_transform(-theta3 + pi/2, 0, l3, 0);
    A4 = A1*A2*A3;
%     A4(1:3, 4)
%     [x, y, z]'
%     x
%     A4(1, 4)
%     assert(abs(x - A4(1, 4)) <= 0)
%     assert(y == A4(2, 4))
%     assert(z == A4(3, 4))
    A2_ = A1*A2;

    z0 = [0, 0, 1]';
    z1 = [0, 0, 1]';
    z2 = [0, 0, 1]';
    O0 = [0, 0, 0]';

    O1 = A1(1:3, 4);
    O2 = A2_(1:3, 4);
    O3 = A4(1:3, 4);


    jacobian = [cross(z0, (O2 - O0)), cross(z1, (O2 - O1)), cross(z2, (O3 - O2));
                z0, z1, z2];

    j_inv = pinv(jacobian);
    theta_dot = j_inv(:, 1:3)*[vx, vy, vz]';
    theta = [theta1; theta2; theta3];

end