function new_points = sample_points(points, sample_dist)
   

    first = points(1,:);
    second = points(2,:);

    dist = norm(first - second);
    ori  = (second - first) / dist;
    
    sum_dist = 0.0;

    new_points = []

    for i = 1:(length(points) - 1)
        
        second = points(i+1,:);

        dist = norm(first - second);
        ori  = (second - first) / dist;

        while (dist > sample_dist)
            new_points = [new_points;first, ori, sum_dist];
            first = ori*sample_dist + first;

            dist = norm(first - second);
            sum_dist = sum_dist + sample_dist;
        end
    end

    new_points = [new_points;first, ori, sum_dist];
end