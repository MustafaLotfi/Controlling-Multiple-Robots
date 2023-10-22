function u=calc_u(~, q, q_d, delta_Q, drp, init_params, j)
consider_collision = true;
constant1 = init_params.constant1;
m = init_params.m;
k_rep = init_params.K_rep(:, j);
l_min = init_params.robots_min_dist;
k = init_params.K(:, j);
rp_min = init_params.rp_min;

if consider_collision
    nr = size(delta_Q, 2);
    Lp = sqrt(sum(delta_Q.^2, 1));
    E_ij = delta_Q./Lp;
    F_rep = [0, 0]';
    for ii=1:nr
        l = Lp(ii);
        if l > l_min
            F0 = [0, 0]';
        else
            F0 = -k_rep.*(constant1+abs(l-l_min)).*E_ij(:,ii);
        end
        F_rep = F_rep + F0;
    end
    
    np = size(drp, 2);
    if np
        Lp = sqrt(sum(drp.^2, 1));
        Ep_ij = drp./Lp;
        for ii=1:np
            lp = Lp(ii);
            if lp > rp_min
                F0 = [0, 0]';
            else
                F0 = -k_rep.*(constant1+abs(lp-rp_min)).*Ep_ij(:,ii);
            end
            F_rep = F_rep + F0;
        end
    end
else
    F_rep = [0, 0]';
end
u = m*(q_d(3:3:6)-k(2:2:4).*(q(2:2:4)-q_d(2:3:6))-k(1:2:4).*(q(1:2:4)...
    -q_d(1:3:6)))+F_rep;