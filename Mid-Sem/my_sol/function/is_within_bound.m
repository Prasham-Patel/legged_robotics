function bool = is_within_bound(pos, u, lmin, lmax)
    radius = ((u(1) - pos(1))^2 + (u(2) - pos(2))^2 + (u(3) - pos(3))^2)^0.5;
    if (radius >= lmin && radius <= lmax)
        bool = true;
    else
        bool = false;
    end
end