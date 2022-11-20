P1 = [-240, 220, 970, 0, 0, 0]';
P2 = [-340, 300, 890, 0, 0, 0]';
P3 = [240, 220, 970, 0, 0, 0]';
P4 = [340, 300, 890, 0, 0, 0]';

nominal_lor = [604.8652, 604.8652, 604.8652, 604.8652, 604.8652, 604.8652]';

config_points = [P1, P2, P3, P4]
length(config_points)
for i = 1:4
    config_points(1:6, i)
    nominal_l(1:6, i) = expected_IK(config_points(1:6, i), 'xyz');
    delta_l_nom(1:6, i) = nominal_l(1:6, i) - nominal_lor;
end
delta_l_nom

for i = 1:4
    P_real(1:6, i) = FK(config_points(1:6, i), nominal_l(1:6, i), 'xyz')
end
P_real

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
sol = cost_function([s, u, l])

sol = lsqnonlin(@cost_function, [s, u, l])

% for i = 1:4
%     L_expected(1:6, i) = expected_IK(P_real(1:6, i), 'xyz')
% end
% l2 = L_expected(1:6, 1).*L_expected(1:6, 1)
% (delta_l_nom(1:6, 1) + nominal_lor)
% cost = L_expected(1:6, 1).*L_expected(1:6, 1) - (delta_l_nom(1:6, 1) + nominal_lor).*(delta_l_nom(1:6, 1) + nominal_lor)