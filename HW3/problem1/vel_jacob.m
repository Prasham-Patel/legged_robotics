%% P = [x y z a b c]' ; euler_angles = string {'zyz' or 'xyz' and so on ...}
function J = vel_jacob(P, euler_angles)
    [L, N, R, s] = IK(P, euler_angles);
    for i = 1:6
        J(i, :) = [N(:, i)', cross((R*s(:, i)), N(:, i))'];
    end
end
