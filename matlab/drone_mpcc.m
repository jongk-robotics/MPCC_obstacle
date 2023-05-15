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


BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'drone_mpcc');     % Set your problemname. If you 
                                                   % skip this, all files will
                                                   % be named "myAcadoProblem"
    time0 = acado.MexInput;
    x0 = acado.MexInputVector;
    S  = acado.MexInputMatrix;
    
    DifferentialState x y z qw qx qy qz vx vy vz T th;          % Differential States:
                                            % xBody, xWheel, vBody, vWheel
                                            
    Control vT wx wy wz vth;                   % Control: 
                                            % dampingForce
                                            
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
                                            % Start at 0s, control in 50
                                            % intervals upto 1s
    SlX = zeros(length(X), 1);
    SlU = [0 0 0 0 -200];
   
    ref = circular(th_(X));   
    pr  = ref(1:3);
    tr  = ref(2:4);
    
    error = cal_error(p_(X), pr, tr);
    el    = error(1:3);
    ec    = error(4:6);
    
    h = [error.', w_(U).'];
    r = zeros(size(h));

    ocp.minimizeLSQ(S, h, r);             % Minimize this Least Squares Term
    ocp.minimizeLSQLinearTerms(SlX, SlU);
    
    ocp.subjectTo( f );                     % Your OCP is always subject to your 
                                            % differential equation
                                           
    
    ocp.subjectTo( 0     <= T <= 15.0 ); 
    ocp.subjectTo( -5.0     <= vT <= 5.0 );
    ocp.subjectTo( -10.0     <= wx <= 10.0 );
    ocp.subjectTo( -10.0     <= wy <= 10.0 );
    ocp.subjectTo( -10.0     <= wz <= 10.0 );
    ocp.subjectTo( 0     <= vth <= 1.0 );
    

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
X = [2.05,-0.05,1.95,1,0,0,0,0,0,0,9.81,0];
ql = 10;
qc = 100;
qw = [0.001, 0.001, 0.001];
S  = diag([ql, ql, ql, qc, qc, qc, qw]);
dt = 0.01;
sim_T  = 30;

times = 0:dt:sim_T;
states = zeros(length(times), length(X));
ref_pos = zeros(length(times), 3);
lag_error = zeros(length(times), 1);
con_error = zeros(length(times), 1);

for i = 1:length(times)
    tic
    out = drone_mpcc_RUN(t, X, S);  
    U   = out.U
    toc
    drone_x   = [p_(X),q_(X),v_(X),T_(X)];
    drone_u   = [vT_(U),w_(U)]

    drone_x   = DroneRK4(drone_x', drone_u', dt)'
    th        = th_(X) + dt*vth_(U)
    X     = [drone_x, th];
    t     = t + dt
    states(i, :) = X;
    
    reference = circular(X(end));
    ref_pos(i, :) = reference(1:3);

    error = cal_error(p_(X)', reference(1:3), reference(4:6))
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
