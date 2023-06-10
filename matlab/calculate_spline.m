function coeffs = calculate_spline(points, sample_distance)
   
    T = sample_distance;
    T2 = T*T;
    T3 = T2*T;

    A = [0, 0, 0, 1;
         T3, T2, T, 1;
         0, 0, 1, 0;
         3*T2, 2*T, 1 0];
    coeffs = [];

    for i = 1:(length(points) - 1)
        first = points(i,:);
        second = points(i+1,:);

        v = [first(1:3);
             second(1:3);
             first(4:6);
             second(4:6)];
        a = A \ v;
        
        coeffs = [coeffs; a(:, 1)', a(:, 2)', a(:, 3)'];
    end
end