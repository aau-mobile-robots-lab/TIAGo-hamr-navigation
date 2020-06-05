%% 
close all
clear all
clc
%% Get data messages from ROS bag

%Load bagfile
bagfolder = "/Users/reiserbalazs/Documents/TIAGo-hamr-navigation/Matlab_simulation/ReadRosbag/Bagfiles/";
bagname = "2020-06-02-18-22-39.bag";
bagpath = bagfolder + bagname;
bag = rosbag(bagpath);

 %% /cmd_vel
 %geometry_msgs/Twist
cmd_topic_select = select(bag, 'Topic', '/nav_vel');
cmd_vel = timeseries(cmd_topic_select, 'Linear.X', 'Angular.Z');

%Start timeseries from 0
cmd_vel.Time = cmd_vel.Time - cmd_vel.Time(1);
Plot_Control_Input(cmd_vel.Time, cmd_vel.Data(:, 1:2), 0, 0.4, -pi/3, pi/3)

%% /robot_pose
%geometry_msgs/PoseWithCovariance
robot_pose_select = select(bag, 'Topic', '/robot_pose');
robot_pose = timeseries(robot_pose_select, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y');
%translate to 0,0
%robot_pose.Data(:,1:2) = robot_pose.Data(:,1:2)-robot_pose.Data(1,1:2);
robot_pose.Time = robot_pose.Time - robot_pose.Time(1);
%PlotRobotPose(robot_pose.Data(:,1), robot_pose.Data(:,2))

%% /global plan
%nav_msgs/Path
gplan_select = select(bag, 'Topic', '/move_base/GlobalPlanner/plan');
gplan_msg = readMessages(gplan_select, 1);
gplan_first = gplan_msg{1}.Poses;
for i = 1:size(gplan_first, 1)
   global_plan(i, 1:2) = [gplan_first(i).Pose.Position.X, gplan_first(i).Pose.Position.Y];
end
%translate to 0,0
%global_plan = global_plan - global_plan(1, :)
%plot(global_plan(:, 1), global_plan(:, 2), 'r', 'LineWidth', 1.5)

%% /move_base/current_goal
%geometry_msgs/PoseStamped
current_goal_select = select(bag, 'Topic', '/move_base/current_goal');
current_goal_msg = readMessages(current_goal_select);
[yaw, pitch, roll] = quat2angle([current_goal_msg{1}.Pose.Orientation.X,current_goal_msg{1}.Pose.Orientation.Y, ...
                                 current_goal_msg{1}.Pose.Orientation.Z, current_goal_msg{1}.Pose.Orientation.W]);
%current_goal_XY = timeseries(current_goal_select, 'Pose.Position.X', 'Pose.Position.Y');
current_goal = [current_goal_msg{1}.Pose.Position.X, current_goal_msg{1}.Pose.Position.X, roll];
%current_goal_XY = timeseries(current_goal_select, 'Pose.Position.X', 'Pose.Position.Y');
%plotArrow(current_goal(1), current_goal(2), current_goal(3), 0.3, 0.15, 0.1, 'g')

PlotRobotPose(robot_pose.Data(:,1), robot_pose.Data(:,2))
hold on
plot(global_plan(:, 1), global_plan(:, 2), 'r', 'LineWidth', 1.5)
hold on
plotArrow(current_goal(1), current_goal(2), current_goal(3), 0.3, 0.15, 0.1, 'g')

%% /mobs
%visualization_msgs/MarkerArray'
MOobs_select = select(bag, 'Topic', '/mobs');
MOobs_msg = readMessages(MOobs_select);
for i  = 1:size(MOobs_msg, 1)
    for j = 1:size(MOobs_msg{i}.Markers, 1)
        MOPoses(i, j, 1:2) = [MOobs_msg{i}.Markers(j).Pose.Position.X, MOobs_msg{i}.Markers(j).Pose.Position.Y];
    end
end

%% /prediction_poses
%geometry_msgs/PoseArray
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