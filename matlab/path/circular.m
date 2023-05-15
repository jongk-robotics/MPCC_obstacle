function ref = circular(theta)
    r = 2;
    angle = theta / r;
    ref = [r*cos(angle);r*sin(angle);2;
           -sin(angle) ;cos(angle)  ;0];