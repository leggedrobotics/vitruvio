% This script reads trajectory data from a rosbag and writes the result
% into a .mat file. The user specifies the path to the trajectory data and
% the name of the .mat file where the data is saved.

clc;
clear all;
close all;

%% Enter path to bag containing trajectory data
pathToTrajectoryData =  '/Users/michaelchadwick/Documents/git/vitruvio/data/trajectory_data/hopper/matlab_defaultHopper.bag';
legCount = 1;
%% Extract the desired 3D vectors from the bag
bag_all = rosbag(pathToTrajectoryData);

t0 = 0.0; %bag_all.StartTime;
T  = bag_all.EndTime;

selectOptions = {'Time', [t0 T] };
bag = select(bag_all, selectOptions{:});

%% base motion
%Base position and orientation
bag_base_pose = select(bag, 'Topic', 'base_pose');
ts_base_pos = timeseries(bag_base_pose, 'Position.X', 'Position.Y', 'Position.Z', ... 
                                        'Orientation.X', 'Orientation.Y', 'Orientation.Z', 'Orientation.W');
% Base velocity                                    
bag_base_twist = select(bag, 'Topic', 'base_twist');
ts_base_vel = timeseries(bag_base_twist, 'Linear.X', 'Linear.Y', 'Linear.Z');

bag_base_acc  = select(bag, 'Topic', 'base_acc');
ts_base_acc = timeseries(bag_base_acc, 'Z');

% Base rotation
motion.quat = [ts_base_pos.Data(:,7) -ts_base_pos.Data(:,4) -ts_base_pos.Data(:,5) -ts_base_pos.Data(:,6)];

%% endeffector motion
bag_foot_0 = select(bag, 'Topic', 'foot_pos_0');
ts_foot_LF = timeseries(bag_foot_0, 'X', 'Y', 'Z');

bag_foot_1 = select(bag, 'Topic', 'foot_pos_1');
ts_foot_RF = timeseries(bag_foot_1, 'X', 'Y', 'Z');

bag_foot_2 = select(bag, 'Topic', 'foot_pos_2');
ts_foot_LH = timeseries(bag_foot_2, 'X', 'Y','Z');

bag_foot_3 = select(bag, 'Topic', 'foot_pos_3');
ts_foot_RH = timeseries(bag_foot_3, 'X', 'Y','Z');

%% endeffector forces
bag_force_0 = select(bag, 'Topic', 'foot_force_0');
ts_force_LF = timeseries(bag_force_0, 'X', 'Y','Z');

bag_force_1 = select(bag, 'Topic', 'foot_force_1');
ts_force_RF  = timeseries(bag_force_1, 'X', 'Y','Z');

bag_force_2 = select(bag, 'Topic', 'foot_force_2');
ts_force_LH  = timeseries(bag_force_2, 'X', 'Y','Z');

bag_force_3 = select(bag, 'Topic', 'foot_force_3');
ts_force_RH  = timeseries(bag_force_3, 'X', 'Y','Z');

motion.t = ts_base_pos.Time; 
motion.dt = motion.t(2)-motion.t(1);

% base acceleration
base_zdd = ts_base_acc.Data(:,1);

% foot position world frame
motion.EE.LF.position = [ts_foot_LF.Data(:,1) ts_foot_LF.Data(:,2) ts_foot_LF.Data(:,3)];
if legCount > 1
motion.EE.RF.position = [ts_foot_RF.Data(:,1) ts_foot_RF.Data(:,2) ts_foot_RF.Data(:,3)];
end
if legCount > 2
motion.EE.LH.position = [ts_foot_LH.Data(:,1) ts_foot_LH.Data(:,2) ts_foot_LH.Data(:,3)];
end
if legCount > 3
motion.EE.RH.position = [ts_foot_RH.Data(:,1) ts_foot_RH.Data(:,2) ts_foot_RH.Data(:,3)];
end

%foot velocity world frame
for i=1:length(ts_foot_LF.Data(:,1))-1
    motion.EE.LF.velocity(i+1,:) = (motion.EE.LF.position(i+1,:)-motion.EE.LF.position(i,:))/motion.dt;
    if legCount > 1
    motion.EE.RF.velocity(i+1,:) = (motion.EE.RF.position(i+1,:)-motion.EE.LF.position(i,:))/motion.dt;
    end
    if legCount > 2
    motion.EE.LH.velocity(i+1,:) = (motion.EE.LH.position(i+1,:)-motion.EE.LF.position(i,:))/motion.dt;
    end
    if legCount > 3
    motion.EE.RH.velocity(i+1,:) = (motion.EE.RH.position(i+1,:)-motion.EE.LF.position(i,:))/motion.dt;
    end
end

% foot force
motion.EE.LF.force = [ts_force_LF.Data(:,1) ts_force_LF.Data(:,2) ts_force_LF.Data(:,3)];
if legCount > 1
motion.EE.RF.force = [ts_force_RF.Data(:,1) ts_force_RF.Data(:,2) ts_force_RF.Data(:,3)];
end
if legCount > 2
motion.EE.LH.force = [ts_force_LH.Data(:,1) ts_force_LH.Data(:,2) ts_force_LH.Data(:,3)];
end
if legCount > 3
motion.EE.RH.force = [ts_force_RH.Data(:,1) ts_force_RH.Data(:,2) ts_force_RH.Data(:,3)];
end

% magnitude of force
for i =1:length(motion.EE.LF.force(:,1))
    motion.EE.LF.force(i,4) = norm(motion.EE.LF.force(i,:),2);
    if legCount > 1
    motion.EE.RF.force(i,4) = norm(motion.EE.RF.force(i,:),2);
    end
    if legCount > 2
    motion.EE.LH.force(i,4) = norm(motion.EE.LH.force(i,:),2);
    end
    if legCount > 3
    motion.EE.RH.force(i,4) = norm(motion.EE.RH.force(i,:),2);
    end
end

motion.base.position = [ts_base_pos.Data(:,1) ts_base_pos.Data(:,2) ts_base_pos.Data(:,3)];
motion.base.velocity = [ts_base_vel.Data(:,1) ts_base_vel.Data(:,2) ts_base_vel.Data(:,3)];
motion.base.acceleration = ts_base_acc.Data(:,1);

save('defaultHopper.mat', '-struct','motion') 
% %% error
% % 
% m = 29.52;   % weight of the robot
% g = 9.81; % gravity acceleration
% startPoint = 150;
% endPoint = 350;
% F_ext = motion.EE.LF.force(startPoint:endPoint,3) + motion.EE.RF.force(startPoint:endPoint,3) + motion.EE.LH.force(startPoint:endPoint,3) + motion.EE.RH.force(startPoint:endPoint,3);
% base_zdd_dynamics = 1/m*F_ext - g;
% % calculate Root mean square error
% base_zdd_error = base_zdd_dynamics - base_zdd(startPoint:endPoint);
% norm_square = norm(base_zdd_error)^2;
% n = size(motion.t(startPoint:endPoint),1); % number of sampled points
% RMSE = sqrt(norm_square/n) 