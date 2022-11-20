[l, s, s_new, u, R, o] = IK([0, 10, 100, deg2rad(15), deg2rad(5) ,0]', 'xyz')
l1 = 20;
l2 = 70;
l3 = 100;

l = transpose(R)*l;

for i = 1:6
    alpha = atan(l(2, i)/l(1, i));
    li1 = [l1*cos(alpha), l1*sin(alpha), 0];
    
    s_knee = s(:, i) + [(-1^i)*l1*cos(alpha), (-1^i)*l1*sin(alpha), 0]';
    s_knee = o + R*s_knee;
    
    alpha = alpha + pi;
    
    li = norm(s_knee - u(:, i));
    gamma = pi - acos((l2^2 + l3^2 - li^2)/(2*l2*l3));
    
    leg = norm(l(:, i));
    
    q = asin((s_knee(3) - s_new(3, i))/(l1));
    p = asin(s_knee(3)/li);
    beta = acos((l2^2 + li^2 - l3^2)/(2*l2*li)) + acos((l1^2 + li^2 - leg^2)/(2*l1*li)) -pi;
    
    %% check if angles are correct
    %% Forward kinematic of leg joints  must give ground contact point

    A0 = [R, s_new(:,i); 0, 0, 0, 1];
    A1 = h_transform(alpha, -pi/2, 20, 0);
    A2 = h_transform(-beta, 0, 70, 0);
    A3 = h_transform(gamma, 0, 100, 0);

    FK = A0*A1*A2*A3;
    
    if (round(FK(1:3, 4) - u(:, i), 5) == [0, 0, 0]')
        Joint_angles = rad2deg([alpha, beta, gamma])
        fprintf("for leg %d are correct \n", i);
        fprintf("\n");
    else
        Joint_angles = rad2deg([alpha, beta, gamma])
        fprintf("ERROR: Joint angles for leg %d are incorrect \n", i);
        break;
    end
end