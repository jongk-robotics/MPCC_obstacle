clear;close all;

addpath path
 
p_  = @(X) X(1:3);
q_  = @(X) X(4:7);
v_  = @(X) X(8:10);
T_  = @(X) X(11);
th_ = @(X) X(12);

vT_  = @(U) U(1);
w_   = @(U) U(2:4);
vth_ = @(U) U(5);


BEGIN_ACADO;                                
    
    acadoSet('problemname', 'drone_mpcc');     
                                                   
                                                   
    time0 = acado.MexInput;
    x0 = acado.MexInputVector;
    S  = acado.MexInputMatrix;

    cx3 = acado.MexInput;
    cx2 = acado.MexInput;
    cx1 = acado.MexInput;
    cx0 = acado.MexInput;

    cy3 = acado.MexInput;
    cy2 = acado.MexInput;
    cy1 = acado.MexInput;
    cy0 = acado.MexInput;

    cz3 = acado.MexInput;
    cz2 = acado.MexInput;
    cz1 = acado.MexInput;
    cz0 = acado.MexInput;
    th_bias = acado.MexInput;


    C = [cx3, cx2, cx1, cx0;
         cy3, cy2, cy1, cy0;
         cz3, cz2, cz1, cz0];
    
    DifferentialState x y z qw qx qy qz vx vy vz T th;          % Differential States:
                                            
    Control vT wx wy wz vth;                   % Control: 
                                            
                                            
    X = [x;y;z;qw;qx;qy;qz;vx;vy;vz;T;th];
    U = [vT;wx;wy;wz;vth];

    drone_x = [p_(X);q_(X);v_(X);T_(X)];  
    drone_u = [vT_(U);w_(U)];

    
    %% Differential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
    
    drone_dx = DroneEuler(drone_x, drone_u);

    for i = 1:length(drone_x)
        f.add(dot(drone_x(i)) == drone_dx(i));    
    end
    f.add(dot(th) == vth);
    
    
    %% Optimal Control Problem
    dt = 0.01;
    N = 30;

    ocp = acado.OCP(0.0, dt*N, N);          % Set up the Optimal Control Problem (OCP)
                                            
                                            
    SlX = zeros(length(X), 1);
    SlU = [0 0 0 0 -200];
    
    th1 = th - th_bias;
    th2 = th1*th1;
    th3 = th2*th1;
    
    pr = C*[th3; th2; th1; 1];
    tr = C*[3*th2; 2*th1; 1; 0];
    
    error = cal_error(p_(X), pr, tr);
    el    = error(1:3);
    ec    = error(4:6);
    
    h = [error.', w_(U).'];
    r = zeros(size(h));

    ocp.minimizeLSQ(S, h, r);             % Minimize this Least Squares Term
    ocp.minimizeLSQLinearTerms(SlX, SlU);
    
    ocp.subjectTo( f );                     % Your OCP is always subject to your 
                                            % differential equation    
    ocp.subjectTo( 0      <= T <= 15.0 ); 
    ocp.subjectTo( -5.0   <= vT <= 5.0 );
    ocp.subjectTo( -10.0  <= wx <= 10.0 );
    ocp.subjectTo( -10.0  <= wy <= 10.0 );
    ocp.subjectTo( -10.0  <= wz <= 10.0 );
    ocp.subjectTo( 0      <= vth <= 1.0 );
    

    algo = acado.RealTimeAlgorithm(ocp, dt);
    algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );
    algo.set('MAX_NUM_ITERATIONS', 2 );
    algo.set('PRINTLEVEL', 0 );
    algo.set('PRINT_COPYRIGHT', 0 );

    controller = acado.Controller( algo);
    controller.init(time0, x0);     % USE THE FREE VARIABLES
    controller.step(time0, x0);

    % simulation
%     identity = acado.OutputFcn();
%     dynamicSystem = acado.DynamicSystem(f, identity);
%     process = acado.Process(dynamicSystem, 'INT_RK45');
% 
%     sim = acado.SimulationEnvironment( 0.0,3.0,process,controller );
% 
%     sim.init( x0 );
   
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. 
                     % You can run the file problemname_ACADO.m as
                     % many times as you want without having to compile again.

%%
t = 0;
X = [0.05,-0.05,0.05,1,0,0,0,0,0,0,9.81,0];
ql = 10000;
qc = 1000;
qw = [0.001, 0.001, 0.001];
S  = diag([ql, ql, ql, qc, qc, qc, qw]);
dt = 0.01;
sim_T  = 30;

start_pos = [0, 0, 0];
end_pos = [10, 10, 10];
obs_list = [5,5,5,1];
points = path_planning(start_pos, end_pos, obs_list)

% theta=0:0.01:2*pi;
% points=[cos(theta')-1, sin(theta'), zeros(length(theta), 1)];

sample_dist = 0.5;
new_points = sample_points(points, sample_dist);
spline = calculate_spline(new_points, sample_dist);

plot_path(new_points, spline, sample_dist)

times = 0:dt:sim_T;
states = zeros(length(times), length(X));
ref_pos = zeros(length(times), 3);
lag_error = zeros(length(times), 1);
con_error = zeros(length(times), 1);


point_idx = 1;

for i = 1:length(times)
    if point_idx < size(new_points, 1)
        point = new_points(point_idx, 1:3);
        next_point = new_points(point_idx+1, 1:3);
        v = check_inside(point', next_point', p_(X)');
        if v > 1.0
            point_idx = point_idx + 1;
        end
        if point_idx >= size(new_points, 1)
            break
        end
    end

    cx = spline(point_idx, 1:4);
    cy = spline(point_idx, 5:8);
    cz = spline(point_idx, 9:12);
    C = [cx;cy;cz];

    th_bias = sample_dist*(point_idx-1);

    tic
    out = drone_mpcc_RUN(t, X, S, cx(1), cx(2), cx(3), cx(4), cy(1), cy(2), cy(3), cy(4), cz(1), cz(2), cz(3), cz(4), th_bias);  
    U   = out.U
    toc
    drone_x   = [p_(X),q_(X),v_(X),T_(X)];
    drone_u   = [vT_(U),w_(U)]

    drone_x   = DroneRK4(drone_x', drone_u', dt)'
    th        = th_(X) + dt*vth_(U)
    X     = [drone_x, th];
    t     = t + dt
    states(i, :) = X;
    
    th1 = th_(X) - th_bias ;
    th2 = th1*th1;
    th3 = th2*th1;
    
    pr = C*[th3; th2; th1; 1];
    tr = C*[3*th2; 2*th1; 1; 0];
    
    reference = straight_line(X(end));
    ref_pos(i, :) = pr;

    error = cal_error(p_(X)', pr, tr)
    lag_error(i,:) = norm(error(1:3));
    con_error(i,:) = norm(error(4:6));
end

%%
pos = states(:, 1:3);
pos_r = ref_pos(:, 1:3);

figure
plot(times, pos, times, ref_pos);
legend(["x" "y" "z" "xr" "yr" "zr"])
xlabel("time (s)")
title("position")

figure
plot(times, pos - ref_pos);
legend(["x" "y" "z"])
xlabel("time (s)")
title("position error")


figure
plot3(pos(:, 1), pos(:, 2), pos(:, 3), pos_r(:, 1), pos_r(:, 2), pos_r(:, 3))
legend(["state" "reference"])
xlabel("time (s)")
title("trajectory")

figure
plot(times, lag_error, times, con_error);
legend(["lag error" "contour error"])
xlabel("time (s)")
ylabel("error (m)")
title("trakcing error")


%% 
save('result/states.mat','states');