%% 
clear all
%% Get data messages from ROS bag

%Load bagfile
bagfolder = "/Users/reiserbalazs/Documents/TIAGo-hamr-navigation/Matlab_simulation/ReadRosbag/Bagfiles/";
bagname = "2020-06-02-18-22-39.bag";
bagpath = bagfolder + bagname;
bag = rosbag(bagpath);

 %% /cmd_vel
cmd_topic_select = select(bag, 'Topic', '/nav_vel');
cmd_vel_linear = timeseries(cmd_topic_select, 'Linear.X');
cmd_vel_angular = timeseries(cmd_topic_select, 'Angular.Z');

%Start timeseries from 0
cmd_vel_linear.Time = cmd_vel_linear.Time - cmd_vel_linear.Time(1);
cmd_vel_angular.Time = cmd_vel_angular.Time - cmd_vel_angular.Time(1);

%% /robot_pose
robot_pose_select = select(bag, 'Topic', '/robot_pose');
robot_pose = timeseries(robot_pose_select, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y');
%translate to 0,0
robot_pose.Data(:,1:2) = robot_pose.Data(:,1:2)-robot_pose.Data(1,1:2);
robot_pose.Time = robot_pose.Time - robot_pose.Time(1);

%% /global plan
gplan_select = select(bag, 'Topic', '/move_base/GlobalPlanner/plan');
gplan_msg = readMessages(gplan_select, 1);
gplan_first = gplan_msg{1}.Poses;
for i = 1:size(gplan_first, 1)
    gplan_x(1, i) = gplan_first(i).Pose.Position.X;
end
for j = 1:size(gplan_first, 1)
    gplan_y(j) = gplan_first(j).Pose.Position.Y;
end
gplan_x = gplan_x - gplan_x(1);
gplan_y = gplan_y - gplan_y(1);

%% /whill/controller/joy

% whill_con_joy_select = select(bag, 'Topic', '/whill/controller/joy');
% joy_cmd_angular = timeseries(whill_con_joy_select);
% joy_cmd_angular.Data = cellfun(@(x) x.Axes(1), readMessages(whill_con_joy_select));
% 
% joy_cmd_linear = timeseries(whill_con_joy_select);
% joy_cmd_linear.Data = cellfun(@(x) x.Axes(2), readMessages(whill_con_joy_select));
% 
% joy_cmd_linear.Time = joy_cmd_linear.Time - joy_cmd_linear.Time(1);
% joy_cmd_angular.Time = joy_cmd_angular.Time - joy_cmd_angular.Time(1);

%% /joy
% joy_select = select(bag, 'Topic', '/joy');

% joy_angular = timeseries(joy_select);
% joy_angular.Data = cellfun(@(x) x.Axes(7), readMessages(joy_select));
% 
% joy_linear = timeseries(joy_select);
% joy_linear.Data = cellfun(@(x) x.Axes(8), readMessages(joy_select));
% 
% joy_linear.Time = joy_linear.Time - joy_linear.Time(1);
% joy_angular.Time = joy_angular.Time - joy_angular.Time(1);

%% /whill/odom
% odom_select = select(bag, 'Topic', '/whill/odom');
% odom_pose = timeseries(odom_select, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y');
% %translate to 0,0
% odom_pose.Data(:,1:2) = odom_pose.Data(:,1:2)-odom_pose.Data(1,1:2);
% odom_pose.Time = odom_pose.Time - odom_pose.Time(1);

%% /amcl_pose
%amcl_select = select(bag, 'Topic', '/amcl_pose');
%amcl_pose = timeseries(amcl_select, 'Pose.Pose.Position.X');
%translate to 0,0
%amcl_pose.Data(:,1:2) = amcl_pose.Data(:,1:2)-amcl_pose.Data(1,1:2);
%amcl_pose.Time = amcl_pose.Time - amcl_pose.Time(1);

%% /whill/imu

% imu_select = select(bag, 'Topic', '/whill/states/imu');
% imu_linear = timeseries(imu_select, 'LinearAcceleration.X', 'LinearAcceleration.Y', 'LinearAcceleration.Z')
% imu_angular = timeseries(imu_select, 'AngularVelocity.X', 'AngularVelocity.Y', 'AngularVelocity.Z');
% imu_linear.Time = imu_linear.Time - imu_linear.Time(1);
% imu_angular.Time = imu_angular.Time - imu_angular.Time(1);

%% /gazebo/model_states
% gazmod_select = select(bag, 'Topic', '/gazebo/model_states');
% gazmod_pose = readMessages(gazmod_select, 1:150:gazmod_select.NumMessages);
% for i = 1:size(gazmod_pose, 1)
%     local_x(i) = gazmod_pose{i}.Pose(3).Position.X;
%     local_y(i) = gazmod_pose{i}.Pose(3).Position.Y;
% end
% local_x = local_x - local_x(1);
% local_y = local_y - local_y(1);

