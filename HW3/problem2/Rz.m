function Rz = Rz(a)
%     syms a
    Rz = [cos(a) -sin(a) 0;
      sin(a) cos(a) 0;
      0 0 1];
end