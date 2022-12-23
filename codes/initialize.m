function init_params=initialize(problem_number, nearest_destination)
%% Some parameters
% Robots mass and radius
m = 0.2;
r = 0.2;
d = r/2;
rp_min = 1.5*r;

% Simulation
t0 = 0;
dt = 0.05;
final_dist_err = 0.02;
i_stop = 10000;
% Lower than this value will stop siumulation

%% Problem #1 parameters, 2 Robots
if problem_number == 1
    % Initial Condition
    x10 = 1;
    dx10 = 0;
    y10 = 1.25;
    dy10 = 0;
    x20 = -0.5;
    dx20 = 0;
    y20 = 1;
    dy20 = 0;
    Q{1} = [x10, dx10, y10, dy10]';
    Q{2} = [x20, dx20, y20, dy20]';

    robots_min_dist = 2.5*r;
    % Desired values
    x1_d = -(r+d);
    dx1_d = 0;
    ddx1_d = 0;
    y1_d = r+d;
    dy1_d = 0;
    ddy1_d = 0;
    x2_d = r+d;
    dx2_d = 0;
    ddx2_d = 0;
    y2_d = r+d;
    dy2_d = 0;
    ddy2_d = 0;
    if nearest_destination
        dist11 = sqrt((Q{1}(1)-x1_d)^2+(Q{1}(3)-y1_d)^2);
        dist21 = sqrt((Q{2}(1)-x1_d)^2+(Q{2}(3)-y1_d)^2);
        dist12 = sqrt((Q{1}(1)-x2_d)^2+(Q{1}(3)-y2_d)^2);
        dist22 = sqrt((Q{2}(1)-x2_d)^2+(Q{2}(3)-y2_d)^2);
        min_dist = min([dist11, dist21, dist12, dist22]);
        if min_dist == dist12 || min_dist == dist21
            x2o = x2_d;
            y2o = y2_d;
            x2_d = x1_d;
            y2_d = y1_d;
            x1_d = x2o;
            y1_d = y2o;
        end
    end
    Q_d = [x1_d, dx1_d, ddx1_d, y1_d, dy1_d, ddy1_d;
        x2_d, dx2_d, ddx2_d, y2_d, dy2_d, ddy2_d]';

    % Important points
    p1 = [-2*(2*r+d), 2*r+d]';
    p2 = [-2*(r+d), 2*r+d]';
    p3 = [-2*(r+d), 0]';
    p4 = [-(r+d), 0]';
    p5 = [0, 0]';
    p6 = [r+d, 0]';
    p7 = [2*(r+d), 0]';
    p8 = [2*(r+d), 2*r+d]';
    p9 = [2*(2*r+d), 2*r+d]';
    points = [p1, p2, p3, p4, p5, p6, p7, p8, p9];

    % Controller
    k = 15;
    kp = k;
    kd = k;
    kpx1 = kp;
    kdx1 = kd;
    kpy1 = kp;
    kdy1 = kd;
    kpx2 = kp;
    kdx2 = kd;
    kpy2 = kp;
    kdy2 = kd;
    K = [kpx1, kdx1, kpy1, kdy1;
        kpx2, kdx2, kpy2, kdy2]';

    k_rep = 12;
    k1_rep = k_rep*[1, 1]';
    k2_rep = k_rep*[1, 1]';
    K_rep = [k1_rep, k2_rep];
    constant1 = 0.5;
end
%% Problem #2 parameters, 9 Robots
if problem_number == 2
    Q{1} = [2, 0, 1, 0]';
    Q{2} = [-1, 0, -2, 0]';
    Q{3} = [0, 0, 0, 0]';
    Q{4} = [1, 0, -1, 0]';
    Q{5} = [-2, 0, -1, 0]';
    Q{6} = [-1, 0, 0, 0]';
    Q{7} = [0, 0, 1, 0]';
    Q{8} = [0, 0, -2, 0]';
    Q{9} = [-1, 0, -1, 0]';
    
    robots_min_dist = 3*r;
    % Desired values
    Q_d = [-1, 0, 0, 1, 0, 0;
        0, 0, 0, 1, 0, 0;
        1, 0, 0, 1, 0, 0;
        -1, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0;
        1, 0, 0, 0, 0, 0;
        -1, 0, 0, -1, 0, 0;
        0, 0, 0, -1, 0, 0;
        1, 0, 0, -1, 0, 0]';
    
    % Important points
    points = [];
    
    % % Controller
    n_robots = size(Q_d, 2);
    k = 20;
    K = k*ones(4, n_robots);
    k_rep = 40;
    K_rep = k_rep*ones(2, n_robots);
    constant1 = 1;
end
%% Gathering parameters
init_params.m = m;
init_params.r = r;
init_params.robots_min_dist = robots_min_dist;
init_params.rp_min = rp_min;

init_params.t_vec = t0;
init_params.dt = dt;
init_params.final_dist_err = final_dist_err;
init_params.i_stop = i_stop;

init_params.Q = Q;
init_params.Q_d = Q_d;

init_params.points = points;

init_params.K = K;
init_params.K_rep = K_rep;
init_params.constant1 = constant1;

init_params.U = cell(1, numel(init_params.Q));

