close all; clc; clear all
sample_dist = 0.1;
points = [0,0,0; 0.1,0,0;0.1,0,0.1;1,2,4;5,2,0;3,3,3];
new_points = sample_points(points, sample_dist)

%%
prev_points = [0,0,0; new_points(1:(end-1),1:3)];
diff = new_points(:, 1:3) - prev_points;
sqrt(sum(diff.*diff, 2))

%% 
figure
hold on

plot3(points(:, 1), points(:, 2), points(:, 3))
plot3(new_points(:, 1), new_points(:, 2), new_points(:, 3))

%% 
spline = calculate_spline(new_points, sample_dist)

times = 0:0.01:(sample_dist*(length(new_points)-1))

sampled_pos = [];
sampled_vel = [];

for i = 1:(length(times) - 1)
    t = times(i);
    idx = fix(t / sample_dist) + 1;
    T = rem(t, sample_dist);
    T2 = T*T;
    T3 = T2*T;

    TT = [T3;T2;T;1];
   
    coeff_x = spline(idx, 1:4);
    coeff_y = spline(idx, 5:8);
    coeff_z = spline(idx, 9:12);

    TT_v = [3*T2;2*T;1;0];

    sampled_pos = [sampled_pos; coeff_x*TT, coeff_y*TT, coeff_z*TT];
    sampled_vel = [sampled_vel; coeff_x*TT_v, coeff_y*TT_v, coeff_z*TT_v];
end

plot3(sampled_pos(:, 1), sampled_pos(:, 2), sampled_pos(:, 3))
legend(["origin" " smapled" "spline"])
    
sqrt(sum(sampled_vel.*sampled_vel, 2))