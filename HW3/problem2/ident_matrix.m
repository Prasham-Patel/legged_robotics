function Jp = ident_matrix(N, R)
    Jp = zeros(6, 42);
    for i = 1:6
        Jp(i, 1+((i-1)*7):i*7) = [N(:, i)'*R, N(:, i)', -1];
    end
end