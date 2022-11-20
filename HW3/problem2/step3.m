%% run step1 and step2 to get points inside the workspace
step1_and_step2

%% Real values
sr1 = [96.6610, 81.7602, 1.0684];
sr2 = [22.2476, 125.2511, -0.5530];
sr3 = [-122.4519 36.6453 4.3547];
sr4 = [-120.6859 -34.4565 -4.9014];
sr5 = [24.7769 -125.0489 -4.8473];
sr6 = [91.3462 -80.9866 0.2515];

sr = [sr1, sr2, sr3, sr4, sr5, sr6]

ur1 = [305.2599 115.0695 2.6210];
ur2 = [-55.2814 322.9819 4.2181];
ur3 = [-244.7954 208.0087 3.9365];
ur4 = [-252.5755 -211.8783 -3.0];
ur5 = [-53.9678 -320.6115 4.3];
ur6 = [302.4266 -109.4351 3.381];

ur = [ur1, ur2, ur3, ur4, ur5, ur6]

lr = [604.4299 607.2473 600.4441 605.9031 604.5251 600.0616];

%% Nominal values
s1 = [92.1597 84.4488 0];
s2 = [27.055 122.037 0];
s3 = [-119.2146 37.5882 0];
s4 = [-119.2146 -37.5882 0];
s5 = [27.055 -122.037 0];
s6 = [92.1597 -84.4488 0];

s = [s1, s2, s3, s4, s5, s6]

u1 = [305.4001 111.1565 0];
u2 = [-56.4357 320.0625 0];
u3 = [-248.9644 208.9060 0];
u4 = [-248.9644 -208.9060 0];
u5 = [-56.4357 -320.0625 0];
u6 = [305.4001 -111.1565 0];

u = [u1, u2, u3, u4, u5, u6]

l = [604.8652 604.8652 604.8652 604.8652 604.8652 604.8652];

delta_s = sr - s;
delat_u = ur - u;
delta_l = lr - l;
dp = [];
for i = 1:6
    dp = [dp, delta_s(1, 3*(i-1) + 1: 3*i), delat_u(1, 3*(i-1) + 1: 3*i), delta_l(i)];
end
% dp = [s-sr, u-ur, lmin-lminr]
for i = 1:length(valid_points)
    P = [valid_points(:, i)', 0, 0, 0]';
    Jv = vel_jacob(P, 'zyz');
    [L, N, R, s] = IK(P, 'zyz');
    Jp = ident_matrix(N, R);
    pos(:, i) = pinv(Jv)*(-Jp*dp');
    RSS(i) = (pos(1, i)^2 + pos(2, i)^2 + pos(3, i)^2)^0.5;
end

points_x = valid_points(1, :);
points_y = valid_points(2, :);
points_z = RSS;

scatter3(points_x, points_y, points_z)


