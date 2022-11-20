function Rx = Rx(c)
%     syms c
    Rx = [1 0 0;
          0 cos(c) -sin(c);
          0 sin(c) cos(c)];
end