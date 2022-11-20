addpath('utility')

v_body_g = 0.16;
beta = 0:0.05:1;
v_leg_g = [];
v_body_g_arr = [];

for i = 1:length(beta)
    v_body_g_arr = [v_body_g_arr, v_body_g];
    v_leg_g = [v_leg_g, (beta(i)*v_body_g)/(1 - beta(i))];
end

figure(1)
plot(v_body_g_arr, v_leg_g)
grid on
title("V(t) vs u(t)")
saveas(figure(1), "V(t)_verses_u(t)", "png")

figure(2)
plot(beta, v_leg_g)
grid on
title("beta vs u(t)")
saveas(figure(2), "beta_verses_u(t)", "png")

