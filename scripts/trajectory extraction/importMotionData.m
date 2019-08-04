% This script reads trajectory data from a rosbag and writes the result
% into a .mat file. The user specifies the path to the trajectory data and
% the name of the .mat file where the data is saved.

clc;
clear;
close all;

%% Enter path to bag containing trajectory data
pathToTrajectoryData =  '/Users/michaelchadwick/Documents/git/vitruvio/data/extractedBags/matlab_ANYmalBear_slowTrot_intermediateTorque.bag';
legCount = 4; % Specify number of legs from 1 to 4.

%% Extract the desired 3D vectors from the bag
bag_all = rosbag(pathToTrajectoryData);

t0 = 0.0; %bag_all.StartTime;
T  = bag_all.EndTime;

selectOptions = {'Time', [t0 T] };
bag = select(bag_all, selectOptions{:});

% Base pose
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

% End effector motion
bag_foot_0 = select(bag, 'Topic', 'foot_pos_0');
ts_foot_LF = timeseries(bag_foot_0, 'X', 'Y', 'Z');

if legCount > 1
    bag_foot_1 = select(bag, 'Topic', 'foot_pos_1');
    ts_foot_RF = timeseries(bag_foot_1, 'X', 'Y', 'Z');
end

if legCount > 2
    bag_foot_2 = select(bag, 'Topic', 'foot_pos_2');
    ts_foot_LH = timeseries(bag_foot_2, 'X', 'Y','Z');
end

if legCount > 3
    bag_foot_3 = select(bag, 'Topic', 'foot_pos_3');
    ts_foot_RH = timeseries(bag_foot_3, 'X', 'Y','Z');
end

% End effector forces
bag_force_0 = select(bag, 'Topic', 'foot_force_0');
ts_force_LF = timeseries(bag_force_0, 'X', 'Y','Z');

if legCount > 1
    bag_force_1 = select(bag, 'Topic', 'foot_force_1');
    ts_force_RF  = timeseries(bag_force_1, 'X', 'Y','Z');
end

if legCount > 2
    bag_force_2 = select(bag, 'Topic', 'foot_force_2');
    ts_force_LH  = timeseries(bag_force_2, 'X', 'Y','Z');
end

if legCount > 3
    bag_force_3 = select(bag, 'Topic', 'foot_force_3');
    ts_force_RH  = timeseries(bag_force_3, 'X', 'Y','Z');
end
motion.t = ts_base_pos.Time; 
motion.dt = motion.t(2)-motion.t(1);

% Base acceleration
base_zdd = ts_base_acc.Data(:,1);

%% Save the extracted bag data into a structure

% End effector position
    motion.trajectoryData.LF.position = [ts_foot_LF.Data(:,1) ts_foot_LF.Data(:,2) ts_foot_LF.Data(:,3)];
if legCount > 1
    motion.trajectoryData.RF.position = [ts_foot_RF.Data(:,1) ts_foot_RF.Data(:,2) ts_foot_RF.Data(:,3)];
end
if legCount > 2
    motion.trajectoryData.LH.position = [ts_foot_LH.Data(:,1) ts_foot_LH.Data(:,2) ts_foot_LH.Data(:,3)];
end
if legCount > 3
    motion.trajectoryData.RH.position = [ts_foot_RH.Data(:,1) ts_foot_RH.Data(:,2) ts_foot_RH.Data(:,3)];
end

% End effector forces.
motion.trajectoryData.LF.force = [ts_force_LF.Data(:,1) ts_force_LF.Data(:,2) ts_force_LF.Data(:,3)];
if legCount > 1
    motion.trajectoryData.RF.force = [ts_force_RF.Data(:,1) ts_force_RF.Data(:,2) ts_force_RF.Data(:,3)];
end
if legCount > 2
    motion.trajectoryData.LH.force = [ts_force_LH.Data(:,1) ts_force_LH.Data(:,2) ts_force_LH.Data(:,3)];
end
if legCount > 3
    motion.trajectoryData.RH.force = [ts_force_RH.Data(:,1) ts_force_RH.Data(:,2) ts_force_RH.Data(:,3)];
end

% Compute magnitude of force
for i =1:length(motion.trajectoryData.LF.force(:,1))
    motion.trajectoryData.LF.force(i,4) = norm(motion.trajectoryData.LF.force(i,:),2);
    if legCount > 1
        motion.trajectoryData.RF.force(i,4) = norm(motion.trajectoryData.RF.force(i,:),2);
    end
    if legCount > 2
        motion.trajectoryData.LH.force(i,4) = norm(motion.trajectoryData.LH.force(i,:),2);
    end
    if legCount > 3
        motion.trajectoryData.RH.force(i,4) = norm(motion.trajectoryData.RH.force(i,:),2);
    end
end

motion.trajectoryData.base.position     = [ts_base_pos.Data(:,1) ts_base_pos.Data(:,2) ts_base_pos.Data(:,3)];

save('ANYmalBearSlowTrotIntermediateTorque.mat', '-struct','motion') 

%% Error 
m = 38.8;   % mass of the robot
g = 9.81; % gravity acceleration

if legCount == 1
    F_ext = motion.trajectoryData.LF.force(:,3);
end

if legCount == 2
    F_ext = motion.trajectoryData.LF.force(:,3) + motion.trajectoryData.RF.force(:,3);
end

if legCount == 3
    F_ext = motion.trajectoryData.LF.force(:,3) + motion.trajectoryData.RF.force(:,3) + motion.trajectoryData.LH.force(:,3);
end

if legCount == 4
    F_ext = motion.trajectoryData.LF.force(:,3) + motion.trajectoryData.RF.force(:,3) + motion.trajectoryData.LH.force(:,3) + motion.trajectoryData.RH.force(:,3);
end

base_zdd_dynamics = 1/m*F_ext - g;
% calculate Root mean square error
base_zdd_error = base_zdd_dynamics - base_zdd;
norm_square = norm(base_zdd_error)^2;
n = size(motion.t,1); % number of sampled points
RMSE = sqrt(norm_square/n) 