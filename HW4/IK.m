%% P = [x y z a b c]' ; euler_angles = string {'zyz' or 'xyz' and so on ...}
%% Input for [a, b, c] is in radians
function [l, s, s_new, u, R, o] = IK(P, euler_angles)
    Rm = 300/2;
    Rf = 480/2;

% radially symetrical, therefor alpha and beta both are 60 degrees
    alpha = 60*pi/180;
    beta = 60*pi/180;
    
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
    s6=[Rm*cos(beta/2) , Rm*sin(beta/2), 0]';
    s4=[-Rm*sin(pi/6-beta/2) , Rm*cos(pi/6-beta/2), 0]';
    s2=[-Rm*sin(pi/6+beta/2) , Rm*cos(pi/6+beta/2), 0]';
    s1=[-Rm*cos(pi/3-beta/2) , -Rm*sin(pi/3-beta/2), 0]';
    s3=[-Rm*cos(pi/3+beta/2) , -Rm*sin(pi/3+beta/2), 0]';
    s5=[Rm*cos(beta/2) , -Rm*sin(beta/2), 0]';
    s = [Rz(-pi/2)*s1 , Rz(-pi/2)*s2 , Rz(-pi/2)*s3 , Rz(-pi/2)*s4 , Rz(-pi/2)*s5 , Rz(-pi/2)*s6];

    %% Calculating lower joint positions wrt. the lower corrdinate frame
    u6=[Rf*cos(alpha/2) , Rf*sin(alpha/2), 0]';
    u4=[-Rf*sin(pi/6-alpha/2) , Rf*cos(pi/6-alpha/2), 0]';
    u2=[-Rf*sin(pi/6+alpha/2) , Rf*cos(pi/6+alpha/2), 0]';
    u1=[-Rf*cos(pi/3-alpha/2) , -Rf*sin(pi/3-alpha/2), 0]';
    u3=[-Rf*cos(pi/3+alpha/2) , -Rf*sin(pi/3+alpha/2), 0]';
    u5=[Rf*cos(alpha/2) , -Rf*sin(alpha/2), 0]';
    
    u = [Rz(-pi/2)*u1 , Rz(-pi/2)*u2 , Rz(-pi/2)*u3 , Rz(-pi/2)*u4 , Rz(-pi/2)*u5 , Rz(-pi/2)*u6];

    %% Put it in the IK equation
    for i = 1:6
        l (: , i) = o + R * s(: , i) - u (: , i);
        s_new(:, i) = o + R * s(: , i);
%         L(i) = norm (l (: , i), 2);
%         N(:, i) = l (: , i)/L(i);
    end

end