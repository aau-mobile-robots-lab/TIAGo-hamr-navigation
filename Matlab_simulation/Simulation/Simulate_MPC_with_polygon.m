function Simulate_MPC_with_polygon (x_ol,x_cl,o_cl,SO_cl_index,SO_polygon,x_ref,N,rob_diameter,xyaxis)
%% Figure setup
figure(100)
fig = gcf; %Current figure handle
fig.Color = 'w';
fig.Units = 'pixel';
fig.OuterPosition = [0 0 1000 1000];
fig.PaperPositionMode = 'auto';

%% Draw simulation

% Footprint of the robot
robot_radius = rob_diameter/2;
draw_ang=0:0.005:2*pi;               % angles for to draw the robot
x_robot = robot_radius*cos(draw_ang);
y_robot = robot_radius*sin(draw_ang);

% arrow triangle parameters
arrow_h = 0.1; arrow_w=0.05;

x_driven = [];
y_driven = [];

step_size = 1;

centroid = [];
k_pos = 1;
[SO_vector, SO_dims] = SO_struct2Matrix(SO_polygon);
n_SO = size(SO_cl_index,2);
for k = 1:size(SO_polygon,2)
    poly_x = SO_vector(k_pos:k_pos+SO_dims(k)-1,1);
    poly_y = SO_vector(k_pos:k_pos+SO_dims(k)-1,2);
    centroid = [centroid; CalculatePolygonCentroid(poly_x, poly_y)];
    k_pos = k_pos+SO_dims(k);
end

for k = 1:step_size:size(x_ol,2)-1 % go through the open loop
    %% Plot SO (Static Obstacles)
    i_pos = 1;
    for i = 1:size(SO_dims,2)
        if SO_dims<=2
            plot(SO_vector(i_pos:i_pos+SO_dims(i)-1,1), SO_vector(i_pos:i_pos+SO_dims(i)-1,2), 'm-o');
            hold on;
            i_pos = i_pos+SO_dims(i);
        else
            poly_x = [SO_vector(i_pos:i_pos+SO_dims(i)-1,1);SO_vector(i_pos,1)];
            poly_y = [SO_vector(i_pos:i_pos+SO_dims(i)-1,2);SO_vector(i_pos,2)];
            plot(poly_x, poly_y, 'm-o');
            hold on;
            i_pos = i_pos+SO_dims(i);
        end
    end
    
    %% plot centroid
    for i = 1:size(SO_polygon,2)
        plot(centroid(i,1), centroid(i,2), 'g*');
        hold on
        %drawCircle(centroid(i,1), centroid(i,2), centroid(i,3), 'g--');
        %hold on
    end
    
    %% Plot Closest obstacle centroids
    
    for i = 1:n_SO
        cl_poses = [];
        cl_size = size(SO_polygon(SO_cl_index(k, i)).point, 2);
        for j = 1:cl_size
            cl_poses = [cl_poses; [SO_polygon(SO_cl_index(k, i)).point(j).x{:}, SO_polygon(SO_cl_index(k, i)).point(j).y{:}]];
        end
        if cl_size > 2
            cl_poses = [cl_poses; cl_poses(1,1:2)];
        end
        plot(cl_poses(:, 1), cl_poses(:,2), '-k', 'LineWidth', 1.5);
    end
    
    %% Plot MO (Moving Obstacles) predictions
    if k < size(x_ol,2)
        for i = 1:size(o_cl,1)
            x_obs_fp = o_cl(i,1,5,1)*cos(draw_ang);
            y_obs_fp = o_cl(i,1,5,1)*sin(draw_ang);
            plot(o_cl(i,2:(N+1),1,k), o_cl(i,2:(N+1),2,k), 'c--*')
            hold on
            for j = 2:(N+1)
                plot(o_cl(i,j,1,k)+x_obs_fp, o_cl(i,j,2,k)+y_obs_fp,'--c', 'LineWidth', 0.5)     % plot robot footprint in predictions
                hold on
            end
        end
    end
    
    %% Plot MO current position
    for i = 1:size(o_cl,1)
        ox1 = o_cl(i,1,1,k);
        ox2 = o_cl(i,1,2,k);
        plotArrow(ox1, ox2, o_cl(i,1,3,k), o_cl(i,1,5,k), arrow_h, arrow_w, 'k');
        hold on
        x_obs_fp = o_cl(i,1,5,1)*cos(draw_ang);
        y_obs_fp = o_cl(i,1,5,1)*sin(draw_ang);
        plot(o_cl(i,1,1,k)+x_obs_fp, o_cl(i,1,2,k)+y_obs_fp,'k')
        hold on
    end
    
    %% Plot reference trajectory
    plot(x_ref(:,1), x_ref(:,2), 'k', 'LineWidth', 1.5)
    hold on
    
    %% Plot the driven (executed) trajectory
    x1 = x_ol(1,k,1); y1 = x_ol(2,k,1); th1 = x_ol(3,k,1);
    x_driven = [x_driven x1];
    y_driven = [y_driven y1];
    plot(x_driven,y_driven,'b','LineWidth', 1.5) % plot exhibited trajectory
    hold on
    
    %% Plot reference trajectory until horizon ends
    % Plot positon on reference trajcetory
    if (k+N <= size(x_ref,1))
        plot(x_ref(k:k+N,1), x_ref(k:k+N, 2), 'g*')
    else
        plot(x_ref(k:end,1), x_ref(k:end, 2), 'g*')
    end
    hold on
    
    %% Plot prediction
    if k < size(x_ol,2)
        plot(x_cl(1:N,1,k), x_cl(1:N,2,k), 'r--*')
        hold on
        for i = 2:N+1
            plot(x_cl(i,1,k)+x_robot, x_cl(i,2,k)+y_robot,'--r')     % plot robot footprint in predictions
            hold on
        end
    end

    %% Plot goal position
    plotArrow(x_ref(end,1), x_ref(end,2), x_ref(end,3), robot_radius, arrow_h, arrow_w, 'g');
    hold on
    
    %% Plot Robot footprint
    plotArrow(x1, y1, th1, robot_radius, arrow_h, arrow_w, 'k');
    hold on
    plot(x1+x_robot,y1+y_robot,'k')      % plot robot circle
    
    hold off
    
    ylabel('$y$-position [m]','interpreter','latex','FontSize', 16)
    xlabel('$x$-position [m]','interpreter','latex','FontSize', 16)
    axis(xyaxis);
    
    %pause(0.1)
    
    box on;
    grid on;
    drawnow
end
%close(gcf)
