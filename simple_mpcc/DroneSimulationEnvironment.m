% Define the Predefined Path 
center = [0, 0];          % Center of the circular path
radius = 15;               % Radius of the circular path
theta = linspace(0, 2*pi, 100);  % Angle parameter
path_x = center(1) + radius*cos(theta);  % X-coordinate of the path
path_y = center(2) + radius*sin(theta);  % Y-coordinate of the path
path_z = theta;  % Z-coordinate of the path

% Define Roll-Pitch-Yaw
path_roll = linspace(0, pi/6, 100);
path_pitch = linspace(0,pi/6, 100);
path_yaw = linspace(0, pi/2, 100);

% Set Initial Conditions
initial_pos = [path_x(1), path_y(1), path_z(1)];  % Initial position
initial_rot = [path_roll(1), path_pitch(1), path_yaw(1)];  % Initial rotation angles (roll, pitch, yaw)

% Set Simulation Parameters
dt = 0.1;  % Time step
duration = 10;  % Duration of simulation (in seconds)
num_steps = duration / dt;  % Number of simulation steps

% Initialize Variables
current_pos = initial_pos;  % Current position
current_rot = initial_rot;  % Current rotation angles

% Create Figure and Axes
figure;
ax = axes;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(ax, 30, 30);  % Set the desired fixed view

% Create Animated Line Object
drone_line = animatedline('LineWidth', 1.5, 'Color', 'r');

% Define the drone vertices
bar1_vertices = [-0.1 -3 -0.05;  % Vertex 1
                  0.1 -3 -0.05;  % Vertex 2
                  0.1  3 -0.05;  % Vertex 3
                 -0.1  3 -0.05;  % Vertex 4
                 -0.1 -3  0.05;  % Vertex 5
                  0.1 -3  0.05;  % Vertex 6
                  0.1  3  0.05;  % Vertex 7
                 -0.1  3  0.05]; % Vertex 8

bar2_vertices = [-3 -0.1 -0.05;  % Vertex 1
                  3 -0.1 -0.05;  % Vertex 2
                  3  0.1 -0.05;  % Vertex 3
                 -3  0.1 -0.05;  % Vertex 4
                 -3 -0.1  0.05;  % Vertex 5
                  3 -0.1  0.05;  % Vertex 6
                  3  0.1  0.05;  % Vertex 7
                 -3  0.1  0.05]; % Vertex 8

% Define the bar faces
faces = [1 2 3 4;   % Face 1
         2 6 7 3;   % Face 2
         6 5 8 7;   % Face 3
         5 1 4 8;   % Face 4
         4 3 7 8;   % Face 5
         1 5 6 2];  % Face 6



% Set the center and radius of the disk
disk1_center = [3,  0, 0.1];
disk2_center = [-3, 0, 0.1];
disk3_center = [0,  3, 0.1];
disk4_center = [0, -3, 0.1];
radius = 1.25;          % Radius of the disk

% Set the coordinate of each disk
theta = linspace(0, 2*pi, 100);
disk1_x = disk1_center(1) + radius*cos(theta);
disk1_y = disk1_center(2) + radius*sin(theta);
disk1_z = disk1_center(3)*ones(size(theta));

disk2_x = disk2_center(1) + radius*cos(theta);
disk2_y = disk2_center(2) + radius*sin(theta);
disk2_z = disk2_center(3)*ones(size(theta));

disk3_x = disk3_center(1) + radius*cos(theta);
disk3_y = disk3_center(2) + radius*sin(theta);
disk3_z = disk3_center(3)*ones(size(theta));

disk4_x = disk4_center(1) + radius*cos(theta);
disk4_y = disk4_center(2) + radius*sin(theta);
disk4_z = disk4_center(3)*ones(size(theta));

% Create Drone Patch Object
bar1_patch = patch('Vertices', bar1_vertices, 'Faces', faces, 'FaceColor', 'black', 'EdgeColor', 'none');
bar2_patch = patch('Vertices', bar2_vertices, 'Faces', faces, 'FaceColor', 'black', 'EdgeColor', 'none');
disk1_patch = patch(disk1_x, disk1_y, disk1_z);
disk2_patch = patch(disk2_x, disk2_y, disk2_z);
disk3_patch = patch(disk3_x, disk3_y, disk3_z);
disk4_patch = patch(disk4_x, disk4_y, disk4_z);

% Simulation Loop
for i = 1:num_steps
    t = i * dt;  % Current time
    
    % Compute the desired position and orientation at the current time step
    desired_pos = [path_x(i), path_y(i), path_z(i)];  % Desired position
    desired_rot = [path_roll(i), path_pitch(i), path_yaw(i)];  % Desired rotation angles
    
    % Calculate the rotation matrix
    rotX = [1 0 0; 0 cos(path_roll(i)) -sin(path_roll(i)); 0 sin(path_roll(i)) cos(path_roll(i))];
    rotY = [cos(path_pitch(i)) 0 sin(path_pitch(i)); 0 1 0; -sin(path_pitch(i)) 0 cos(path_pitch(i))];
    rotZ = [cos(path_yaw(i)) -sin(path_yaw(i)) 0; sin(path_yaw(i)) cos(path_yaw(i)) 0; 0 0 1];
    R = rotZ * rotY * rotX;
    
    % Rotate the vertices of the cube
    rotated_bar1_vertices = (R * bar1_vertices')';
    rotated_bar2_vertices = (R * bar2_vertices')';
    
    rotated_disk1_vertices = (R * [disk1_x; disk1_y; disk1_z])';
    rotated_disk2_vertices = (R * [disk2_x; disk2_y; disk2_z])';
    rotated_disk3_vertices = (R * [disk3_x; disk3_y; disk3_z])';
    rotated_disk4_vertices = (R * [disk4_x; disk4_y; disk4_z])';

    % Translate the vertices of the cube
    translated_bar1_vertices = rotated_bar1_vertices + desired_pos;
    translated_bar2_vertices = rotated_bar2_vertices + desired_pos;
    
    translated_disk1_vertices = rotated_disk1_vertices + desired_pos;
    translated_disk2_vertices = rotated_disk2_vertices + desired_pos;
    translated_disk3_vertices = rotated_disk3_vertices + desired_pos;
    translated_disk4_vertices = rotated_disk4_vertices + desired_pos;
    
    % Update the current position and orientation of the drone
    current_pos = desired_pos;
    current_rot = desired_rot;
    
    % Update the drone cube position and orientation
    bar1_vertices_updated = translated_bar1_vertices;
    bar2_vertices_updated = translated_bar2_vertices;
    
    disk1_vertices_updated = translated_disk1_vertices;
    disk2_vertices_updated = translated_disk2_vertices;
    disk3_vertices_updated = translated_disk3_vertices;
    disk4_vertices_updated = translated_disk4_vertices;
    
    set(bar1_patch, 'Vertices', bar1_vertices_updated);
    set(bar2_patch, 'Vertices', bar2_vertices_updated);
    
    set(disk1_patch, 'Vertices', disk1_vertices_updated, 'FaceColor', 'none', 'EdgeColor', 'blue');
    set(disk2_patch, 'Vertices', disk2_vertices_updated, 'FaceColor', 'none', 'EdgeColor', 'blue');
    set(disk3_patch, 'Vertices', disk3_vertices_updated, 'FaceColor', 'none', 'EdgeColor', 'blue');
    set(disk4_patch, 'Vertices', disk4_vertices_updated, 'FaceColor', 'none', 'EdgeColor', 'blue');
    
    pause(0.01);
    
    % Store the current position for visualization
    addpoints(drone_line, current_pos(1), current_pos(2), current_pos(3));
    
    % Update the plot
    drawnow;
end
