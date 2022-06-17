function [vars, init_params]=simulate_system()
init_params = initialize();
r = init_params.r;

Q = init_params.Q;
n_robots = numel(Q);

Q_d = init_params.Q_d;

points = init_params.points;

t_vec = init_params.t_vec;
U = init_params.U;
final_dits_err = init_params.final_dist_err;
dt = init_params.dt;
i_stop = init_params.i_stop;

i = 0;
r_located = false;
r_collisions = cell(1, n_robots);
while true
    Rs_located = [];
    for j=1:n_robots
        distj_end = sqrt(sum((Q{j}(1:2:4,end)-Q_d(1:3:6, j)).^2));
        if distj_end > final_dits_err
            % R1 & R2 distance
            delta_Q = [];
            for ii=1:n_robots
                if ii ~= j
                    delta_Q = [delta_Q, Q{ii}(1:2:4,end)-Q{j}(1:2:4,end)];
                end
            end
            rjn_dist = sqrt(sum(delta_Q.^2, 1));
            
            % Robots & points distances
            if numel(points)
                d_rjp = points-Q{j}(1:2:4, end);
                % Collision check
                rjp_c = sqrt(sum(d_rjp.^2, 1)) < r;
            else
                d_rjp = [];
                rjp_c = false;
            end
            rjrn_c = sqrt(sum(delta_Q.^2, 1)) < (r+r);
            r_collisions{j} = [r_collisions{j}, any(rjrn_c) || any(rjp_c)];
            % Robot 1
            
            % State equations
            [q_new, u] = solver(t_vec(end), Q{j}(:, end), Q_d(:,j),...
                delta_Q, d_rjp, init_params, j);
            Q{j} = [Q{j}, q_new];
            U{j} = [U{j}, u];
            r_located = false;
        else
            r_located = true;
        end
        Rs_located = [Rs_located, r_located];
    end
    if all(Rs_located)
        break
    end
    t_vec = [t_vec, t_vec(end)+dt];
    if i > i_stop
        break
    end
    i = i + 1;
end

vars.t_vec = t_vec;
vars.Q = Q;
vars.Q_d = Q_d;
vars.U = U;
vars.r_collisions = r_collisions;
vars.points = points;