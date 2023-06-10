clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'drone_mpcc_2');     % Set your problemname. If you 
                                                   % skip this, all files will
                                                   % be named "myAcadoProblem"
    time0 = acado.MexInput;
    x0 = acado.MexInputVector;
    
    DifferentialState x y z qw qx qy qz vx vy vz t;          % Differential States:
                                            % xBody, xWheel, vBody, vWheel
                                            
    Control T wx wy wz vt;                   % Control: 
                                            % dampingForce
                                            
    Disturbance R;                          % Disturbance: 
                                            % roadExcitation

    p = [x;y;z];
    X = [p;qw;qx;qy;qz;vx;vy;vz];  
    U = [T;wx;wy;wz];

    
    %% Differential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
    
    dX = DroneEuler(X, U);

    for i = 1:length(X)
        f.add(dot(X(i)) == dX(i));    
    end
    f.add(dot(t) == vt);
    
    
    %% Optimal Control Problem
    h = 0.01;
    N = 30;

    ocp = acado.OCP(0.0, h*N, N);          % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 50
                                            % intervals upto 1s
    Slx = zeros(length(x), 1);
    Slu = [0 0 0 0 -1];
   
    ref_p = takeoff(t);   
    ocp.minimizeLSQ((p - ref_p).');             % Minimize this Least Squares Term
    ocp.minimizeLSQLinearTerms(Slx, Slu);
    
    ocp.subjectTo( f );                     % Your OCP is always subject to your 
                                            % differential equation
                                           
    
    ocp.subjectTo( 0     <= T <= 15.0 ); 
    ocp.subjectTo( 0     <= vt <= 1.0 );

    algo = acado.RealTimeAlgorithm(ocp, h);
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
x = [0,0,0,1,0,0,0,0,0,0,0];
dt = 0.01;
T  = 10;

times = 0:dt:T;
states = zeros(length(times) + 1, length(x));
states(1, :) = x;

for i = 1:length(times)
    tic
    out = drone_mpcc_2_RUN(t, x);  
    toc
    drone_x   = x(1:(end - 1));
    drone_u   = out.U(1:4);

    drone_x   = DroneRK4(drone_x', drone_u', dt)'
    theta     = x(end) + dt*out.U(5);
    x     = [drone_x, theta];
    t     = t + dt
    states(i + 1, :) = x;
end

%%
plot(times, states(2:end, 1:3))
   



