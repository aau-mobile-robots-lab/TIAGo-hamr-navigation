%% 
close all
clear all
clc
%% Get data messages from ROS bag

%Load bagfile
%bagfolder = "/Users/reiserbalazs/Documents/TIAGo-hamr-navigation/Matlab_simulation/ReadRosbag/Bagfiles/";
%bagname = "2020-06-02-18-22-39.bag";
bagfolder = "G:\Bags\SO\SO_Polygon\";
bagname = "2020-06-03-14-37-41.bag";

% bagfolder = "/Users/reiserbalazs/Desktop/Corridor/Poly/";
% bagname = "2020-06-04-16-08-31.bag";
% bagfolder = "/Users/reiserbalazs/Desktop/Corridor/Poly/";
% bagname = "2020-06-04-11-38-12.bag";
bagpath = bagfolder + bagname;
bag = rosbag(bagpath);
%%

mat = dir("G:\Bags\Corridor\PalNav\*.bag");
for i = 1:length(mat)
    bpath = fullfile('G:\Bags','\Corridor','\PalNav\',mat(i).name);
    bagarray(i) = rosbag(bpath);
end

%% Testing
%for z = 1:length(bagarray)     %Start by 12(SOPoly)
%     clearvars -except bagarray bag metrics z metricsSO1


    %%% /cmd_vel

    %geometry_msgs/Twist
%     cmd_topic_select = select(bagarray(z), 'Topic', '/nav_vel');
    cmd_topic_select = select(bag, 'Topic', '/nav_vel');
    cmd_vel = timeseries(cmd_topic_select, 'Linear.X', 'Angular.Z');
    real_cmd_vel_time = cmd_vel.Time;
    
    %Start timeseries from 0
    cmd_vel.Time = cmd_vel.Time - cmd_vel.Time(1);

    %metrics.EndTime(z-1) = cmd_vel.Time(end);
    %%% /robot_pose
    %geometry_msgs/PoseWithCovariance
%     robot_pose_select = select(bagarray(z), 'Topic', '/robot_pose');
    robot_pose_select = select(bag, 'Topic', '/robot_pose');
    robot_pose = timeseries(robot_pose_select, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y');
    
    %translate to time 0
    %robot_pose.Time = robot_pose.Time - robot_pose.Time(1);
    
    %%% /global plan
    %nav_msgs/Path
%     gplan_select = select(bagarray(z), 'Topic', '/move_base/GlobalPlanner/plan');
    gplan_select = select(bag, 'Topic', '/move_base/GlobalPlanner/plan');
    gplan_msg = readMessages(gplan_select, 1);
    gplan_first = gplan_msg{1}.Poses;
    for i = 1:size(gplan_first, 1)
        global_plan(i, 1:2) = [gplan_first(i).Pose.Position.X, gplan_first(i).Pose.Position.Y];
%         plots.global_planx(z) = gplan_first(i).Pose.Position.X;
%         plots.global_plany(z) = gplan_first(i).Pose.Position.Y;
    end
    %gnplom = global_plan(:,1);
     %= global_plan;%global_plan(:,1);
    %plots.global_plany(z) = global_plan(:,2);
    %translate to 0,0

    %%% GOAL
    %goal is the last entry of the global path
    goal =  [global_plan(end, 1), global_plan(end, 2)];

    %%% /move_base/current_goal
    %geometry_msgs/PoseStamped
    %current_goal_select = select(bag, 'Topic', '/move_base_simple/goal');
    %current_goal_msg = readMessages(current_goal_select);
    %[yaw, pitch, roll] = quat2angle([current_goal_msg{1}.Pose.Orientation.X,current_goal_msg{1}.Pose.Orientation.Y, ...
    %                                 current_goal_msg{1}.Pose.Orientation.Z, current_goal_msg{1}.Pose.Orientation.W]);
    %current_goal = [current_goal_msg{1}.Pose.Position.X, current_goal_msg{1}.Pose.Position.Y, roll];

    %%% /mobs
    %visualization_msgs/MarkerArray'
    %simply reads in all poses of each moving obstacle
%     MOobs_select = select(bagarray(z), 'Topic', '/mobs');
    MOobs_select = select(bag, 'Topic', '/mobs');
    MOobs_msg = readMessages(MOobs_select);
% %     plots.MOobs(z) = MOobs_msg.Markers;
    MO_times = timeseries(MOobs_select).Time;
    for i  = 1:size(MOobs_msg, 1)
        for j = 1:size(MOobs_msg{i}.Markers, 1)
            MOPoses(i, j, 1:2) = [MOobs_msg{i}.Markers(j).Pose.Position.X, MOobs_msg{i}.Markers(j).Pose.Position.Y];
        end
    end
%     plots.MOPoses(z) = MOPoses;
    %%% /prediction_poses
    %geometry_msgs/PoseArray
    %Reads in all the poses in the horizon of the robot
%     pred_poses_select = select(bagarray(z), 'Topic', '/prediction_poses');
    pred_poses_select = select(bag, 'Topic', '/prediction_poses');
    pred_poses_msg = readMessages(pred_poses_select);
    for i  = 1:size(pred_poses_msg,1)
        for j = 1:size(pred_poses_msg{i}.Poses, 1)
            [yaw, pitch, roll] = quat2angle([pred_poses_msg{i}.Poses(j).Orientation.X, pred_poses_msg{i}.Poses(j).Orientation.Y, ...
                        pred_poses_msg{i}.Poses(j).Orientation.Z, pred_poses_msg{i}.Poses(j).Orientation.W]);
            prediction_poses(i, j, 1:3) = [pred_poses_msg{i}.Poses(j).Position.X, pred_poses_msg{i}.Poses(j).Position.Y, roll];
            %plotArrow(prediction_poses(i,j, 1), prediction_poses(i,j, 2), prediction_poses(i,j, 3), 0.1, 0.02, 0.01, 'r');
            %plotArrow(1, 1, 2*pi, 0.1, 0.2, 0.01, 'g')
            %hold on
        end
    end

    %%% driven paths
    %global plan distance
    gl_plan_driven_dist = 0;
    for i = 2:size(global_plan, 1)
        gl_plan_driven_dist = gl_plan_driven_dist + ...
            sqrt((global_plan(i, 1)-global_plan(i-1, 1))^2+(global_plan(i, 2)-global_plan(i-1, 2))^2);
    end

    %robot_pose, robot driven distance
    rb_pose_driven_dist = 0;
    for i = 2:size(robot_pose.Data,1)
        rb_pose_driven_dist = rb_pose_driven_dist + ...
            sqrt((robot_pose.Data(i, 1)-robot_pose.Data(i-1, 1))^2+(robot_pose.Data(i, 2)-robot_pose.Data(i-1, 2))^2);
    end
%     metrics.gplandist(z) = gl_plan_driven_dist;
%     metrics.rplandist(z) = rb_pose_driven_dist;
    %%% distance between global and driven
    %error between the robot and global plan
    global_robot_distance_error = [];
    stepsize = floor(size(robot_pose.Data,1)/size(global_plan, 1));
    for i = 1:stepsize:size(robot_pose.Data,1)
        dist_min = inf;
        for j = 1:size(global_plan, 1)
            dist = sqrt((robot_pose.Data(i, 1)-global_plan(j, 1))^2+(robot_pose.Data(i, 2)-global_plan(j, 2))^2);
            if dist < dist_min
                dist_min = dist;
            end
        end
        global_robot_distance_error = [global_robot_distance_error, dist_min];
    end
%     plots.grde(z) = global_robot_distance_error;
%     metrics.d2gplan(z) = mean(global_robot_distance_error);
    %%% TIME
    % 
    starttime = 0;
    i = 1;
    while starttime == 0    %checks for cmd_vel.Data before starting time reading
        if cmd_vel.Data(i, 1) > 0
            starttime = real_cmd_vel_time(i);
        end
        i = i + 1;
    end

    for i = 1:size(robot_pose.Data, 1)
        dist(i) = sqrt((robot_pose.Data(i, 1)-goal(1))^2+(robot_pose.Data(i, 2)-goal(2))^2);
    end
    [a, aa] = min(dist);
    ts_close_to_goal = robot_pose.Time(aa);  %time when robot is close enough to the goal point.
    
    for i = 1:size(cmd_vel.Data,1)
        if (real_cmd_vel_time(i) > ts_close_to_goal && cmd_vel.Data(i, 1) == 0) | (i == length(cmd_vel.Data))
            endtime = real_cmd_vel_time(i);
            break   %if 
        end
    end

    runtime = endtime - starttime;
%     metrics.EndTime(z) = runtime;
    %%% Center of MO distance to robot
    % robot_MO_distance_error = zeros(size(robot_pose.Data,1), size(MOobs_msg{1}.Markers, 1));
    h = 1;
    robot_MO_distance_error = zeros(size(MOobs_msg, 1), size(MOobs_msg{1}.Markers, 1));
    for i = 1:size(MOobs_msg, 1)
        for k = h:size(robot_pose.Data, 1)
            if robot_pose.Time(k) > MO_times(i)
                break
            end
        end
        h = k;
        pose = [robot_pose.Data(k, 1), robot_pose.Data(k, 2)];
        for j = 1:size(MOobs_msg{i}.Markers, 1)
           robot_MO_distance_error(i,j) = sqrt((MOPoses(i,j,1)-pose(1))^2+(MOPoses(i,j,2)-pose(2))^2);
        end
    end
%     metrics.MO_dist1(z) = mean(robot_MO_distance_error(:,1));
%     metrics.MO_dist2(z) = mean(robot_MO_distance_error(:,2));
%     metrics.MO_dist3(z) = mean(robot_MO_distance_error(:,3));
%     metrics.MO_dist4(z) = mean(robot_MO_distance_error(:,4));
%     z
% end
%% Distance from static obstacles




%% Plot
% Plot map

zz2 = 1:length(global_plan(:,1));
zz1 = 1:length(robot_pose.Data(:,1));
figure(1)
surf([robot_pose.Data(:,1) robot_pose.Data(:,1)], ...
    [robot_pose.Data(:,2) robot_pose.Data(:,2)], ... 
    [zz1(:) zz1(:)], ... 
    'FaceColor', 'none','EdgeColor', 'interp','LineWidth', 2);
view(2);
%plot(robot_pose.Data(:,1), robot_pose.Data(:,2), 'b', 'LineWidth', 1)
hold on
% plot(global_plan(:, 1), global_plan(:, 2), 'r', 'LineWidth', 1)
surf([global_plan(:, 1) global_plan(:, 1)], ...
    [global_plan(:,2) global_plan(:,2)], ... 
    [zz2(:) zz2(:)], ... 
    'FaceColor', 'none','EdgeColor', 'interp','LineWidth', 2);
% for j = 1:size(MOobs_msg{i}.Markers, 1)
%     plot(MOPoses(:,j,1), MOPoses(:,j,2), 'k*')
%     hold on
% end
fig = gcf; %Current figure handle
fig.Color = 'w';
fig.Units = 'normalized';
fig.OuterPosition = [0 0.5 0.5 0.5];
axis equal
title('MO, Global path, Driven trajectory', 'interpreter','latex','FontSize', 16)
xlabel('X position (m)', 'interpreter','latex','FontSize', 16)
ylabel('Y position (m)', 'interpreter','latex','FontSize', 16)
legend('Global plan','Robot Pose','Moving Obstacles')
grid on

figure(2)
fig = gcf; %Current figure handle
fig.Color = 'w';
fig.Units = 'normalized';
fig.OuterPosition = [0 0 0.5 0.5];
fig.PaperPositionMode = 'auto';
subplot(2,1,1)
stairs(cmd_vel.Time, cmd_vel.Data(:, 1),'b','linewidth',1.5);
ylabel('$v$ (m/s)', 'interpreter','latex','FontSize', 14)
title('Control input', 'interpreter','latex','FontSize', 16)
grid on
subplot(2,1,2)
stairs(cmd_vel.Time,cmd_vel.Data(:, 2),'b','linewidth',1.5);
xlabel('time (seconds)', 'interpreter','latex','FontSize', 14)
ylabel('$\omega$ (rad/s)', 'interpreter','latex','FontSize', 14)

grid on

figure(3)
plot(global_robot_distance_error, 'b', 'LineWidth', 1)
fig = gcf; %Current figure handle
fig.Color = 'w';
fig.Units = 'normalized';
fig.OuterPosition = [0.5 0.7 0.5 0.3];
title('Error distance between initial global plan and robot position', 'interpreter','latex','FontSize', 16)
xlabel('TimeStep', 'interpreter','latex','FontSize', 14)
ylabel('Error distance (m)', 'interpreter','latex','FontSize', 14)


figure(4)
fig = gcf; %Current figure handle
fig.Color = 'w';
fig.Units = 'normalized';
fig.OuterPosition = [0.5 0.4 0.5 0.3];
for j = 1:size(MOobs_msg{i}.Markers, 1)
    plot(robot_MO_distance_error(:, j), 'LineWidth', 1)
    hold on
end
title('Distance between the robot and the Moving Obstacles', 'interpreter','latex','FontSize', 16)
xlabel('TimeStep', 'interpreter','latex','FontSize', 14)
ylabel('Error distance (m)', 'interpreter','latex','FontSize', 14)
legend('test')




    









