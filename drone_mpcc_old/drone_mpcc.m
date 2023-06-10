clear;close all;
addpath trajectory

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'drone_mpcc');% Set your problemname. If you 
                                            % skip this, all files will
                                            % be named "myAcadoProblem"
                                       
    MexInput x0 y0 z0 qw0 qx0 qy0 qz0 vx0 vy0 vz0 t0;
    DifferentialState x y z qw qx qy qz vx vy vz t;
    Control T wx wy wz vt;                              % Control input

    h = 0.01;
    N = 30;
    
    p = [x;y;z];
    X = [p;qw;qx;qy;qz;vx;vy;vz];  
    U = [T;wx;wy;wz];
    
    %% Diferential Equation
    f = acado.DiscretizedDifferentialEquation(h); % Set the differential equation object
                                                  % 0.01 is the step length
    dX = DroneEuler(X, U);

    for i = 1:length(X)
        f.add(next(X(i)) == X(i) + h*dX(i));    
    end
    f.add(next(t) == t + h*vt);

    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, h*N, N);         % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 50
                                            % intervals upto 10s
    ref_p = takeoff(t);                                        
    ocp.minimizeLagrangeTerm( 10*(p - ref_p)'*(p - ref_p) - vt);        % Minimize the consumed energy
    
    ocp.subjectTo( f );                     % Optimize with respect to your differential equation
    ocp.subjectTo( 'AT_START', t ==  t0 ); % s(0) = 0
    ocp.subjectTo( 'AT_START', x ==  x0 );
    ocp.subjectTo( 'AT_START', y ==  y0 );
    ocp.subjectTo( 'AT_START', z ==  z0 );
    ocp.subjectTo( 'AT_START', qw ==  qw0 );
    ocp.subjectTo( 'AT_START', qx ==  qx0 );
    ocp.subjectTo( 'AT_START', qy ==  qy0 );
    ocp.subjectTo( 'AT_START', qz ==  qz0 );
    ocp.subjectTo( 'AT_START', vx ==  vx0 );
    ocp.subjectTo( 'AT_START', vy ==  vy0 );
    ocp.subjectTo( 'AT_START', vz ==  vz0 );

    ocp.subjectTo( 0     <= T <= 15.0 ); 
    ocp.subjectTo( 0     <= vt <= 1.0 );

    %% Optimization Algorithm
    algo = acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP
%     algo.set( 'HESSIAN_APPROXIMATION', 'EXACT_HESSIAN' );
%     algo.set( 'KKT_TOLERANCE', 1e-10 );     % Set a custom KKT tolerance

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

%% Run the test
p0 = [0;0;0];
q0 = [1;0;0;0];
v0 = [0;0;0];
X0 = [p0;q0;v0];
t0 = 0;

tic
out = drone_mpcc_RUN(2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
toc

t = out.STATES(:, 1);
p = out.STATES(:, 2:4);
% theta = out.STATES(:, 4);
% 
% ref_x = zeros(length(t), 1);
% 
% for i = 1:length(t)
%     ref_x(i) = trajectory(theta(i));
% end
% 
figure
plot(t, p)
legend(["x" "y" "z"])
% 
% figure
% plot(t, out.STATES(:, 4), t, out.STATES(:, 5))
% legend(["t" "vt"])
