%% dp = [s, u, l]
function cost = cost_function(old_dp)
    dp = old_dp';
    s = [dp(1:3), dp(4:6), dp(7:9), dp(10:12), dp(13:15), dp(16:18)];
    u = [dp(19:21), dp(22:24), dp(25:27), dp(28:30), dp(31:33), dp(34:36)];
    lor = dp(37:42);
   
P_real = [ -251.7684 -356.6639  251.7772  356.6937
  230.8419  314.8349  230.7416  314.5984
  965.1418  878.9933  965.1636  879.0661
    0.0161    0.0243    0.0159    0.0239
    0.0174    0.0273   -0.0174   -0.0273
   -0.0001   -0.0003    0.0001    0.0003];



delta_l_nom = [ 483.1094  478.1231  384.5680  334.7393
  377.9257  326.9587  417.8904  386.0148
  372.5933  318.6413  434.3580  409.6909
  446.8880  423.9427  504.5241  506.3977
  462.9084  446.7652  499.8029  499.4327
  493.8575  492.8200  396.3744  351.6419];
    for i = 1: length(P_real)
        for i = 1:4
            L_expected(1:6, i) = expected_IK(P_real(1:6, i), 'xyz');
            cost(i) =norm(L_expected(1:6, i).*L_expected(1:6, i) - (delta_l_nom(1:6, i) + lor(i)).*(delta_l_nom(1:6, i) + lor(i)));
        end
        
    end   
end