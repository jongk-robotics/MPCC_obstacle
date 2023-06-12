function pos_list = path_planning(start_pos, end_pos, obslist)
    p = 5;
    q = 3;
    ka = 5;
    kr = 100;
    
    cur_x = start_pos;
    pos_list = [cur_x];
    for i = 1:10000
        if norm(cur_x - end_pos) < 0.05
            break
        end
    
        vel = potential_field(cur_x, end_pos, p, ka, obslist, q, kr);
        
        len = length(pos_list);
        if len > 10
            v = var(pos_list(len-10:end, :));
            if norm(v) < 0.003
                vel = rand(1, 3);
            end
        end
    
        if norm(vel) > 1
            vel =  vel / norm(vel);
        end
        
        cur_x = cur_x + 0.1*vel;
        pos_list = [pos_list; cur_x];
    end
    
    figure
    plot3(pos_list(:,1), pos_list(:,2), pos_list(:,3))
end

function Uatt = attractive_func(x, x_goal, p, ka)
    d = sqrt((x-x_goal)*(x-x_goal)');
    if d <= p
        Uatt = ka*d*d;
    else
        Uatt = ka*d;
    end
end

function Fatt = attractive_field(x, x_goal, p, ka)
    d = sqrt((x-x_goal)*(x-x_goal)');
    if d <= p
        Fatt = 2*ka*(x_goal - x);
    else
        Fatt = ka*(x_goal - x) / (d + 1e-5);
    end
end

function Urep = repulsive_func(x, x_obs, r_obs, q, kr)
    d = sqrt((x-x_obs)*(x-x_obs)') - r_obs;
    if d <= 0
        Urep = 1;
    elseif d <= q
        Urep = kr*exp(-d*d);
    else
        Urep = 0;
    end
end

function Frep = repulsive_field(x, x_obs, r_obs, q, kr)
    diff = x-x_obs;
    d = sqrt(diff*diff') - r_obs;
    if d <= 0
        Frep = 0;
    elseif d <= q
        Frep = 2*kr*d*diff/sqrt(diff*diff')*exp(-d*d);
    else
        Frep = [0,0,0];
    end
end

function U = potential_func(x, x_goal, p, ka, obs_list, q, kr)
    U_att = attractive_func(x, x_goal, p, ka);
    U_rep = 0;
    for i=1:size(obs_list, 1)
        x_obs = obs_list(i, 1:3);
        r_obs = obs_list(i, 4);
        U_rep = U_rep + repulsive_func(x, x_obs, r_obs, q, kr);
    end

    U = U_att + U_rep;
end

function F = potential_field(x, x_goal, p, ka, obs_list, q, kr)
    F_att = attractive_field(x, x_goal, p, ka);
    F_rep = [0,0, 0];
    for i=1:size(obs_list, 1)
        x_obs = obs_list(i, 1:3);
        r_obs = obs_list(i, 4);
        F_rep = F_rep + repulsive_field(x, x_obs, r_obs, q, kr);
    end

    F = F_att + F_rep;
end