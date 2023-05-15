function error = cal_error(p, pr, tr)
    e  = p - pr;
    el = (e'*tr)*tr;
    ec = e - el;

    error = [el;ec];

