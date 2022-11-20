function L, N = IK(P, euler_angles)
    Rm = 300/2;
    Rf = 500/2;

% radially symetrical, therefor alpha and beta both are 60 degrees
    alpha = 60*pi/180;
    beta = 60*pi/180;
    
    o=P(1:3,1); 
    angles = [deg2rad(P(4)) deg2rad(P(5)) deg2rad(P(6))];

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
    s1=[Rm*cos(beta/2) , Rm*sin(beta/2), 0]';
    s2=[-Rm*sin(pi/6-beta/2) , Rm*cos(pi/6-beta/2), 0]';
    s3=[-Rm*sin(pi/6+beta/2) , Rm*cos(pi/6+beta/2), 0]';
    s4=[-Rm*cos(pi/3-beta/2) , -Rm*sin(pi/3-beta/2), 0]';
    s5=[-Rm*cos(pi/3+beta/2) , -Rm*sin(pi/3+beta/2), 0]';
    s6=[Rm*cos(beta/2) , -Rm*sin(beta/2), 0]';
    s = [s1 , s2 , s3 , s4 , s5 , s6];

%% Calculating lower joint positions wrt. the lower corrdinate frame
    u1=[Rf*cos(alpha/2) , Rf*sin(alpha/2), 0]';
    u2=[-Rf*sin(pi/6-alpha/2) , Rf*cos(pi/6-alpha/2), 0]';
    u3=[-Rf*sin(pi/6+alpha/2) , Rf*cos(pi/6+alpha/2), 0]';
    u4=[-Rf*cos(pi/3-alpha/2) , -Rf*sin(pi/3-alpha/2), 0]';
    u5=[-Rf*cos(pi/3+alpha/2) , -Rf*sin(pi/3+alpha/2), 0]';
    u6=[Rf*cos(alpha/2) , -Rf*sin(alpha/2), 0]';

    u = [u1 , u2 , u3 , u4 , u5 , u6];

%% Put it in the IK equation
    for i = 1:6
        l (: , i) = o + R * s(: , i) - u (: , i);
        L(i) = norm (l (: , i));
        N(i) = l (: , i)/L(i)
    end

end