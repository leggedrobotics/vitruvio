%% Plot towr rosbags in matlab
%
% The bags can be generated using
% towr_ros/src/exe/rosbag_geom_msg_extractor.cc.
%
% Author: Alexander Winkler

clc;
clear all;
close all;

%% List of bags containing motion data
miniPronk =  '~/Documents/thesis/trajectory_data/mini/matlab_mini_flat_pronk_2,1_3,6.bag';

speedyGallop = '~/Documents/thesis/trajectory_data/speedy/matlab_speedy_flat_gallop_35,0_10,0.bag';
speedyStairs = '~/Documents/thesis/trajectory_data/speedy/matlab_speedy_stairs_walk_2,9_4,8.bag';

universalTrot = '~/Documents/thesis/trajectory_data/universal/payload/matlab_universalPL_flat_trot_7,4_8,8.bag';
universalStairs = '~/Documents/thesis/trajectory_data/universal/payload/matlab_universalPL_stairs_walk_3,1_7,4.bag';
universalVersatile = 0;

massivoWalk = '~/Documents/thesis/trajectory_data/massivo/matlab_massivo_flat_walk_2,8_7,0.bag';
massivoStairs = '~/Documents/thesis/trajectory_data/massivo/matlab_massivo_stairs_overlapwalk_6,2_15,8.bag';

centaurWalk = '~/Documents/thesis/trajectory_data/centaur/matlab_centaur_flat_walk_2,8_7,0.bag';
centaurStairs = '~/Documents/thesis/trajectory_data/centaur/matlab_centaur_stairs_overlapwalk_6,2_15,8.bag';


%% Extract the desired 3D vectors from the bag
filePath = centaurStairs;
bag_all = rosbag(filePath);

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


%% 
motion.t = ts_base_pos.Time; 
motion.dt = motion.t(2)-motion.t(1);

% base acceleration
base_zdd = ts_base_acc.Data(:,1);

% foot position world frame
motion.EE.LF.position = [ts_foot_LF.Data(:,1) ts_foot_LF.Data(:,2) ts_foot_LF.Data(:,3)];
motion.EE.RF.position = [ts_foot_RF.Data(:,1) ts_foot_RF.Data(:,2) ts_foot_RF.Data(:,3)];
motion.EE.LH.position = [ts_foot_LH.Data(:,1) ts_foot_LH.Data(:,2) ts_foot_LH.Data(:,3)];
motion.EE.RH.position = [ts_foot_RH.Data(:,1) ts_foot_RH.Data(:,2) ts_foot_RH.Data(:,3)];

%foot velocity world frame
for i=1:length(ts_foot_LF.Data(:,1))-1
    motion.EE.LF.velocity(i+1,:) = (motion.EE.LF.position(i+1,:)-motion.EE.LF.position(i,:))/motion.dt;
    motion.EE.RF.velocity(i+1,:) = (motion.EE.RF.position(i+1,:)-motion.EE.LF.position(i,:))/motion.dt;
    motion.EE.LH.velocity(i+1,:) = (motion.EE.LH.position(i+1,:)-motion.EE.LF.position(i,:))/motion.dt;
    motion.EE.RH.velocity(i+1,:) = (motion.EE.RH.position(i+1,:)-motion.EE.LF.position(i,:))/motion.dt;
end

% foot force
motion.EE.LF.force = [ts_force_LF.Data(:,1) ts_force_LF.Data(:,2) ts_force_LF.Data(:,3)];
motion.EE.RF.force = [ts_force_RF.Data(:,1) ts_force_RF.Data(:,2) ts_force_RF.Data(:,3)];
motion.EE.LH.force = [ts_force_LH.Data(:,1) ts_force_LH.Data(:,2) ts_force_LH.Data(:,3)];
motion.EE.RH.force = [ts_force_RH.Data(:,1) ts_force_RH.Data(:,2) ts_force_RH.Data(:,3)];

% magnitude of force
for i =1:length(motion.EE.LF.force(:,1))
    motion.EE.LF.force(i,4) = norm(motion.EE.LF.force(i,:),2);
    motion.EE.RF.force(i,4) = norm(motion.EE.RF.force(i,:),2);
    motion.EE.LH.force(i,4) = norm(motion.EE.LH.force(i,:),2);
    motion.EE.RH.force(i,4) = norm(motion.EE.RH.force(i,:),2);
end

motion.base.position = [ts_base_pos.Data(:,1) ts_base_pos.Data(:,2) ts_base_pos.Data(:,3)];
motion.base.velocity = [ts_base_vel.Data(:,1) ts_base_vel.Data(:,2) ts_base_vel.Data(:,3)];
motion.base.acceleration = ts_base_acc.Data(:,1);

save('centaurStairs.mat', '-struct','motion') 
% %% error
% % 
% m = 29.52;   % weight of the robot
% g = 9.81; % gravity acceleration
% F_ext = foot_LF.force(:,3) + foot_RF.force(:,3) + foot_LH.force(:,3) + foot_RH.force(:,3);
% base_zdd_dynamics = 1/m*F_ext - g;
% % calculate Root mean square error
% base_zdd_error = base_zdd_dynamics - base_zdd;
% norm_sqare = norm(base_zdd_error)^2;
% n = size(t,1); % number of sampled points
% RMSE = sqrt(norm_sqare/n) 
