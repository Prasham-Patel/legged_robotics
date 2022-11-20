%% P = [x y z a b c]' ; euler_angles = string {'zyz' or 'xyz' and so on ...}
%% Input for [a, b, c] is in radians
function [L, N, R, s] = expected_IK(P, euler_angles)
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
    %% Nominal values
    s1 = [92.1597 84.4488 0];
    s2 = [27.055 122.037 0];
    s3 = [-119.2146 37.5882 0];
    s4 = [-119.2146 -37.5882 0];
    s5 = [27.055 -122.037 0];
    s6 = [92.1597 -84.4488 0];
    
    s = [s1', s2', s3', s4', s5', s6']

    %% Calculating lower joint positions wrt. the lower corrdinate frame
    %% Nominal Values
    u1 = [305.4001 111.1565 0];
    u2 = [-56.4357 320.0625 0];
    u3 = [-248.9644 208.9060 0];
    u4 = [-248.9644 -208.9060 0];
    u5 = [-56.4357 -320.0625 0];
    u6 = [305.4001 -111.1565 0];
    
    u = [u1', u2', u3', u4', u5', u6']

%% Put it in the IK equation
    for i = 1:6
        l (: , i) = o + R * s(: , i) - u (: , i);
        L(i) = norm (l (: , i), 2);
        N(:, i) = l (: , i)/L(i);
    end

end