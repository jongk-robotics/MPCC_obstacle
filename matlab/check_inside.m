function v = check_inside(p1, p2, c)
    p1c = c - p1;
    p1p2 = p2 - p1;

    unit = p1p2 / norm(p1p2);
    v = p1c'*unit / norm(p1p2);
end