function Simulate_MPC_multiple_varying_obs (x_ol,x_cl,o_cl,x_goal,N,rob_diameter)
%% Figure setup
figure(100)
fig = gcf; %Current figure handle
fig.Color = 'w';
fig.Units = 'normalized';
fig.OuterPosition = [0 0 0.5 1];
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

for k = 1:step_size:size(x_ol,2) % go through the open loop
    
    % Plot obstacle prediction
    if k < size(x_ol,2)
        for i = 1:size(o_cl,1)
            x_obs_fp = o_cl(i,1,5,1)*cos(draw_ang);
            y_obs_fp = o_cl(i,1,5,1)*sin(draw_ang);
            plot(o_cl(i,2:(N+1),1,k), o_cl(i,2:(N+1),2,k), 'c--*')
            hold on
            for j = 2:(N+1)
                plot(o_cl(i,j,1,k)+x_obs_fp, o_cl(i,j,2,k)+y_obs_fp,'--c', 'LineWidth', 0.5);     % plot robot footprint in predictions
                hold on
            end
        end
    end
    
    % Plot obstacle current position
    for i = 1:size(o_cl,1)
        ox1 = o_cl(i,1,1,k);
        ox2 = o_cl(i,1,2,k);
        plotArrow(ox1, ox2, o_cl(i,1,3,k), o_cl(i,1,5,k), arrow_h, arrow_w, 'k');
        hold on
        x_obs_fp = o_cl(i,1,5,1)*cos(draw_ang);
        y_obs_fp = o_cl(i,1,5,1)*sin(draw_ang);
        plot(o_cl(i,1,1,k)+x_obs_fp, o_cl(i,1,2,k)+y_obs_fp,'k');
        hold on
    end
    
    
    % Plot the driven (executed) trajectory
    x1 = x_ol(1,k,1); y1 = x_ol(2,k,1); th1 = x_ol(3,k,1);
    x_driven = [x_driven x1];
    y_driven = [y_driven y1];
    
    plot(x_driven,y_driven,'b','LineWidth', 1.5); % plot exhibited trajectory
    hold on 
    
    % Plot prediction
    if k < size(x_ol,2)
        plot(x_cl(1:N,1,k), x_cl(1:N,2,k), 'r--*')
        hold on
        for i = 2:N+1
            plot(x_cl(i,1,k)+x_robot, x_cl(i,2,k)+y_robot,'--r');     % plot robot footprint in predictions
            hold on
        end
    end

    % Plot goal position
    plotArrow(x_goal(1), x_goal(2), x_goal(3), robot_radius, arrow_h, arrow_w, 'g')
    hold on
    
    % Plot Robot footprint
    plotArrow(x1, y1, th1, robot_radius, arrow_h, arrow_w, 'k');
    hold on
    plot(x1+x_robot,y1+y_robot,'k');      % plot robot circle
    
    hold off
    
    ylabel('$y$-position [m]','interpreter','latex','FontSize', 16)
    xlabel('$x$-position [m]','interpreter','latex','FontSize', 16)
    axis([-.2 x_goal(1)+1 -.2 x_goal(2)+1])
    
    %pause(0.1)
    
    box on;
    grid on;
    drawnow
end
%close(gcf)
