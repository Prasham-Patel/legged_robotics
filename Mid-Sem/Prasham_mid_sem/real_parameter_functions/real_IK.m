%% P = [x y z a b c]' ; euler_angles = string {'zyz' or 'xyz' and so on ...}
%% Input for [a, b, c] is in radians
function [L, N, R, s] = real_IK(P, euler_angles)
    Rm = 250/2;
    Rf = 650/2;

% radially symetrical, therefor alpha and beta both are 60 degrees
    alpha = 40*pi/180;
    beta = 80*pi/180;
    
    o=P(1:3,1); 
    angles = [P(4) P(5) P(6)];

%% Calculating Rotation Matrix from Euler Angles
    R = [1 0 0; 0 1 0; 0 0 1];
    i = 1;
    while i <= 3
        if euler_angles(i) == 'x'
            R = R*Rx(angles(i));%[1,0,0;0,cos(angles(i)),-sin(angles(i));0,sin(angles(i)),cos(angles(i))];
        end
        if euler_angles(i) == 'y'
            R = R*Ry(angles(i));%[cos(angles(i)),0,sin(angles(i));0,1,0;-sin(angles(i)),0,cos(angles(i))];
        end
        if euler_angles(i) == 'z'
            R = R*Rz(angles(i));%[cos(angles(i)),-sin(angles(i)),0;sin(angles(i)),cos(angles(i)),0;0,0,1];
        end
        i = i+1;
    end

    %% Calculating upper joint positions wrt. the upper coordinate frame
    %% Real values
    sr1 = [96.6610, 81.7602, 1.0684];
    sr2 = [22.2476, 125.2511, -0.5530];
    sr3 = [-122.4519 36.6453 4.3547];
    sr4 = [-120.6859 -34.4565 -4.9014];
    sr5 = [24.7769 -125.0489 -4.8473];
    sr6 = [91.3462 -80.9866 0.2515];

    s = [sr1', sr2', sr3', sr4', sr5', sr6']

    %% Calculating lower joint positions wrt. the lower corrdinate frame
    %% Real Values
    ur1 = [305.2599 115.0695 2.6210];
    ur2 = [-55.2814 322.9819 4.2181];
    ur3 = [-244.7954 208.0087 3.9365];
    ur4 = [-252.5755 -211.8783 -3.0];
    ur5 = [-53.9678 -320.6115 4.3];
    ur6 = [302.4266 -109.4351 3.381];
    
    u = [ur1', ur2', ur3', ur4', ur5', ur6']


%% Put it in the IK equation
    for i = 1:6
        l (: , i) = o + R * s(: , i) - u (: , i);
        L(i) = norm (l (: , i), 2);
        N(:, i) = l (: , i)/L(i);
    end

end