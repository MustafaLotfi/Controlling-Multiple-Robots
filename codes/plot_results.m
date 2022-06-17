function plot_results(vars, params, static_plot, dynamic_plot)
t_vec = vars.t_vec;
Q = vars.Q;
Q_d = vars.Q_d;
U = vars.U;
r_collisions = vars.r_collisions;
points = vars.points;
r = params.r;
n_robots = numel(Q);
r_x_plot = linspace(-r, r, 20);
r_y_plot = sqrt(r^2-r_x_plot.^2);

%% Plotting
if static_plot
    for i=1:n_robots
        q = Q{i};
        q_d = Q_d(1:3:6,i);
        n = size(q,2);
        figure;
        subplot(2,2,[1,2])
        plot(t_vec(1:n), q(1,:), 'k', 'LineWidth', 2, 'DisplayName', 'x')
        hold on
        plot([t_vec(1), t_vec(end)], [q_d(1), q_d(1)], ':k',...
            'DisplayName', 'x_d')
        hold off
        xlabel('t (sec)')
        ylabel('x (m)')
        title(['Robot #', num2str(i), ' trajectory'])
        legend show
        
        subplot(2,2,[3,4])
        plot(t_vec(1:n), q(3,:), 'k', 'LineWidth', 2, 'DisplayName', 'y')
        hold on
        plot([t_vec(1), t_vec(end)], [q_d(2), q_d(2)], ':k',...
            'DisplayName', 'y_d')
        hold off
        xlabel('t (sec)')
        ylabel('y (m)')
        legend show
        
        figure;
        u = U{i};
        nu = size(u, 2);
        subplot(2,2,[1,2])
        plot(t_vec(1:nu), u(1,:), 'k', 'LineWidth', 2)
        xlabel('t (sec)')
        ylabel('u_x')
        title(['Robot #', num2str(i), ' inputs'])
        subplot(2,2,[3,4])
        plot(t_vec(1:nu), u(2,:), 'k', 'LineWidth', 2)
        xlabel('t (sec)')
        ylabel('u_y')
    end
end

colors = rand(n_robots, 3);
figure;
if numel(points)
    plot(points(1,:), points(2,:), 'DisplayName', 'Wall')
end
hold on
for i=1:n_robots
    q = Q{i};
    q_d = Q_d(1:3:6,i);
    plot(q(1,:), q(3,:), 'color', colors(i,:), 'DisplayName',...
        ['R_', num2str(i)])
    hold on
    plot(q(1,1), q(3,1), 'p', 'color', colors(i,:), 'MarkerSize', 10,...
        'DisplayName', ['R_{', num2str(i), ',0}'])
    
    plot(q(1,end)+r_x_plot, [q(3,end)+r_y_plot;q(3,end)-r_y_plot],...
        'color', colors(i,:), 'LineWidth', 2, 'DisplayName',...
        ['R_', num2str(i)])
    plot(q_d(1)+r_x_plot, [q_d(2)+r_y_plot;q_d(2)-r_y_plot], ':',...
        'color', [0.3, 0.3, 0.3])
end
hold off
xlabel('x')
xlabel('y')
legend show
axis equal
hold off

n_end = 0;
for i=1:n_robots
    n = size(Q{i}, 2);
    if n > n_end
        n_end = n;
    end
end
if dynamic_plot
    for i=1:n_end
        figure(6)
        for j=1:n_robots
            q = Q{j};
            n = size(q, 2);
            q_d = Q_d(1:3:6,j);
            plot(q_d(1)+r_x_plot, [q_d(2)+r_y_plot;q_d(2)-r_y_plot],...
                ':','color', [0.3, 0.3, 0.3])
            hold on
            if numel(points)
                plot(points(1,:), points(2,:), 'DisplayName', 'Wall')
            end
            plot(q(1,:), q(3,:), ':', 'color', colors(j,:))
            axis equal
            xlabel('x (m)')
            ylabel('y (m)')
            title('Dynamic trajectory of robot 1 and robot 2')
            if i < n
                if r_collisions{j}(i)
                    text(0, 0 , 'Collision', 'FontSize', 20)
                end
                plot(q(1,i)+r_x_plot,...
                    [q(3,i)+r_y_plot;q(3,i)-r_y_plot],...
                    'color', colors(j,:), 'LineWidth', 2)
            else
                plot(q(1,end)+r_x_plot,...
                    [q(3,end)+r_y_plot;q(3,end)-r_y_plot],...
                    'color', colors(j,:), 'LineWidth', 2)
            end
        end
        hold off
        pause(0.01)
    end
end