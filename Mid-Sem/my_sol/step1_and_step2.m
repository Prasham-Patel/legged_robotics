%% Robot Parameters
Rm = 250/2;
Rf = 650/2;
alpha = deg2rad(40); 
beta = deg2rad(85); 
lmin = 604.8652;
lmax = 1100;

z0 = 800;

%% Calculate lower joint positions (u) wrt. the lower coordinate frame
u1 = [Rf*cos(alpha/2), Rf*sin(alpha/2), 0]';
u2 = [Rf*cos(alpha/2), -Rf*sin(alpha/2), 0]';
u3 = [0, Rf, 0]';
u4 = [0 , -Rf, 0]';
u5 = [-Rf*cos(alpha/2), Rf*sin(alpha/2), 0]';
u6 = [-Rf*cos(alpha/2), -Rf*sin(alpha/2), 0]';

%% Sphere Center Co-ordinates
u = [u1 u2 u3 u4 u5 u6];

%% Calculate upper joint positions (s) wrt. the upper coordinate frame
s1 = [Rm*cos(alpha/2), Rm*sin(alpha/2), 0]';
s2 = [Rm*cos(alpha/2), -Rm*sin(alpha/2), 0]';
s3 = [0, Rm, 0]';
s4 = [0 , -Rm, 0]';
s5 = [-Rm*cos(alpha/2), Rm*sin(alpha/2), 0]';
s6 = [-Rm*cos(alpha/2), -Rm*sin(alpha/2), 0]';

%% joint positions w.r.t to end effector
s = [s1 s2 s3 s4 s5 s6];

%% Find the valid points for workspace
%% by comparing with the equation of boundary of COW
valid_points = [];
for z0 = 650:20:1000
    for i = -800:20:800
        for j = -800:20:800
            P = [i, j, z0, 0, 0, 0]';
            bool = true;
            for k = 1:6
                % check if point lies within all boundry curves
                % is_within_boundry function compares if the point is inside
                % the boundry curves
                bool = bool && is_within_bound(P(1:3)+s(:, k), u(:, k), lmin, lmax);
            end
            if bool
                valid_points = [valid_points, P(1:3)];
            end
        end
    end
end

points_x = valid_points(1, :);
points_y = valid_points(2, :);
points_z = valid_points(3, :);

% plot the points within workspace
scatter3(points_x,points_y, points_z)


