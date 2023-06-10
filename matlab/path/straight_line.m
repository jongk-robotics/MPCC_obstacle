function ref = straight_line(theta)
    start_pos = [0;0;0];
    end_pos = [8;8;8];

    vec = end_pos - start_pos;
    vec = vec / sqrt(vec'*vec);

    ref = [vec*theta;vec];