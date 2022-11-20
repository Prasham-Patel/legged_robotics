function Ry = Ry(b)
%     syms b
    Ry = [cos(b) 0 sin(b);
      0 1 0;
      -sin(b) 0 cos(b)];
end