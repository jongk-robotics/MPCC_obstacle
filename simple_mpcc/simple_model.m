function next_x = simple_model(t,x,u,p,w)
    next_x(1) =  x + h*vx;  
    next_x(2) =  vx + h*u(1);
    next_x(3) =  t + h*vt;
    next_x(4) =  vt + h*u(2);