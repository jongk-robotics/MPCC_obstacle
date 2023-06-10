clear;

BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    acadoSet('problemname', 'simple_mpcc');% Set your problemname. If you 
                                            % skip this, all files will
                                            % be named "myAcadoProblem"
                                       
    x0 = acado.MexInput;
    v0 = acado.MexInput;
   
    DifferentialState x;
    DifferentialState v;
    DifferentialState t;
    DifferentialState vt;

    Control u;                              % Control input
    Control ut;

    h = 0.01;

    X = [x;vt];
    X0 = [x0; v0];

    A = [0 1; 0 0];
    B = [0; 1];
    
    
    %% Diferential Equation
    f = acado.DiscretizedDifferentialEquation(h); % Set the differential equation object
                                                  % 0.01 is the step length
   
    f.add(next(X) == h*(A*X + B*u));    
    f.add(next(t) == t + h*vt); 
    f.add(next(vt) == vt + h*ut); 
    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, 10.0, 50);         % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s, control in 50
                                            % intervals upto 10s
                                            
    ocp.minimizeLagrangeTerm( 100*(x-trajectory(t)) * (x-trajectory(t)) -vt );        % Minimize the consumed energy
    
    ocp.subjectTo( f );                     % Optimize with respect to your differential equation
    ocp.subjectTo( 'AT_START', t ==  0.0 ); % s(0) = 0
    ocp.subjectTo( 'AT_START', vt ==  0.0 );
    ocp.subjectTo( 'AT_START', x ==  x0 );
    ocp.subjectTo( 'AT_START', v ==  v0 );
    ocp.subjectTo( -0.01 <= u <= 1.3 );     % path constraint on speed
    ocp.subjectTo( 0     <= ut <= 1.0 ); 
    
    %% Optimization Algorithm
    algo =acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP
    algo.set( 'HESSIAN_APPROXIMATION', 'EXACT_HESSIAN' );
    algo.set( 'DISCRETIZATION_TYPE', 'MULTIPLE_SHOOTING')
    algo.set( 'KKT_TOLERANCE', 1e-10 );     % Set a custom KKT tolerance

    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.

%% Run the test
tic
out = simple_mpcc_RUN(0, 0);
toc

t = out.STATES(:, 1);
x = out.STATES(:, 2);
vx = out.STATES(:, 3);
theta = out.STATES(:, 4);

ref_x = zeros(length(t), 1);

for i = 1:length(t)
    ref_x(i) = trajectory(theta(i));
end

figure
plot(t, x, t, ref_x)
legend(["x" "ref_x"])

figure
plot(t, out.STATES(:, 4), t, out.STATES(:, 5))
legend(["t" "vt"])
