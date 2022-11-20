%% P0 = [x y z a b c]' ; Lg = [l1, ,l2, l3, l4, l5, l6]' 
%% euler_angles = string {'zyz' or 'xyz' and so on ...}
%% Input for [a, b, c] is in radians
function Pos = real_FK(P0, Lg, euler_angles)
    P(:, 1) = P0;
    i = 2;
    Dp = 1;
    e = 0.000001;
    while Dp > e
        J = vel_jacob(P(:, i-1), euler_angles);
        angles = P(4:6, i-1);
        R = [1 0 0; 0 1 0; 0 0 1];
        j = 1;
        
        while j <= 3
            if euler_angles(j) == 'x'
                R = R*Rx(angles(j));
                B(:, j) = R(:, 1);
            end
            if euler_angles(j) == 'y'
                R = R*Ry(angles(j));
                 B(:, j) = R(:, 2);
            end
            if euler_angles(j) == 'z'
                R = R*Rz(angles(j));
                B(:, j) = R(:, 3);
            end
            j = j+1;
        end
        T = [eye(3), zeros(3, 3); zeros(3, 3), B];
        [L, N, Rn, s] = real_IK(P(:, i-1), euler_angles);
        Dl = Lg - L';
        P(:, i) = P(:, i-1) + pinv(J*T)*Dl; % using pinv to avoid singularity error
        Dp = norm(P(:, i) - P(:, i-1), 2);
        i = i+1;
    end
    Pos = P(:, end);
end