clear rosbag_wrapper;
clear ros.Bag;
clear all
close all

%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
%bag = ros.Bag.load('example.bag');
bag = ros.Bag.load('rosbag_proto_190827_2019-08-27-11-09-18.bag');
bag.info()

%% Read all messages on a few topics
topic1 = '/anymal_lowlevel_controller/actuator_readings';
topic2 = '/state_estimator/quadruped_state';
topic3 = '/sensors/imu';
topic4 = '/log/loco/leftFore/shouldBeGrounded';
topic5 = '/state_estimator/pose_in_odom';

actuator_messages = bag.readAll({topic1})';
quadruped_state_messages = bag.readAll({topic2})';
sensor_messages = bag.readAll({topic3})';
groundedLF_messages = bag.readAll({topic4})';
pose_in_odom_messages = bag.readAll({topic5})';

for i = 1:length(actuator_messages)
    joint.time(i,1) = actuator_messages{i,1}.readings(1).state.header.stamp.time;
    
    % Joint position measured
    joint.LF.meas.q(i,1) = actuator_messages{i,1}.readings(1).state.joint_position;
    joint.LF.meas.q(i,2) = actuator_messages{i,1}.readings(2).state.joint_position;
    joint.LF.meas.q(i,3) = actuator_messages{i,1}.readings(3).state.joint_position;
    
    joint.RF.meas.q(i,1) = actuator_messages{i,1}.readings(4).state.joint_position;
    joint.RF.meas.q(i,2) = actuator_messages{i,1}.readings(5).state.joint_position;
    joint.RF.meas.q(i,3) = actuator_messages{i,1}.readings(6).state.joint_position;
    
    joint.LH.meas.q(i,1) = actuator_messages{i,1}.readings(7).state.joint_position;
    joint.LH.meas.q(i,2) = actuator_messages{i,1}.readings(8).state.joint_position;
    joint.LH.meas.q(i,3) = actuator_messages{i,1}.readings(9).state.joint_position;

    joint.RH.meas.q(i,1) = actuator_messages{i,1}.readings(10).state.joint_position;
    joint.RH.meas.q(i,2) = actuator_messages{i,1}.readings(11).state.joint_position;
    joint.RH.meas.q(i,3) = actuator_messages{i,1}.readings(12).state.joint_position;
    
    % Joint position desired
    
    joint.LF.des.q(i,1) = actuator_messages{i,1}.readings(1).commanded.position;
    joint.LF.des.q(i,2) = actuator_messages{i,1}.readings(2).commanded.position;
    joint.LF.des.q(i,3) = actuator_messages{i,1}.readings(3).commanded.position;
    
    joint.RF.des.q(i,1) = actuator_messages{i,1}.readings(4).commanded.position;
    joint.RF.des.q(i,2) = actuator_messages{i,1}.readings(5).commanded.position;
    joint.RF.des.q(i,3) = actuator_messages{i,1}.readings(6).commanded.position;
    
    joint.LH.des.q(i,1) = actuator_messages{i,1}.readings(7).commanded.position;
    joint.LH.des.q(i,2) = actuator_messages{i,1}.readings(8).commanded.position;
    joint.LH.des.q(i,3) = actuator_messages{i,1}.readings(9).commanded.position;

    joint.RH.des.q(i,1) = actuator_messages{i,1}.readings(10).commanded.position;
    joint.RH.des.q(i,2) = actuator_messages{i,1}.readings(11).commanded.position;
    joint.RH.des.q(i,3) = actuator_messages{i,1}.readings(12).commanded.position;
    
    % Joint velocity measured
    joint.LF.meas.qdot(i,1) = actuator_messages{i,1}.readings(1).state.joint_velocity;
    joint.LF.meas.qdot(i,2) = actuator_messages{i,1}.readings(2).state.joint_velocity;
    joint.LF.meas.qdot(i,3) = actuator_messages{i,1}.readings(3).state.joint_velocity;
    
    joint.RF.meas.qdot(i,1) = actuator_messages{i,1}.readings(4).state.joint_velocity;
    joint.RF.meas.qdot(i,2) = actuator_messages{i,1}.readings(5).state.joint_velocity;
    joint.RF.meas.qdot(i,3) = actuator_messages{i,1}.readings(6).state.joint_velocity;
    
    joint.LH.meas.qdot(i,1) = actuator_messages{i,1}.readings(7).state.joint_velocity;
    joint.LH.meas.qdot(i,2) = actuator_messages{i,1}.readings(8).state.joint_velocity;
    joint.LH.meas.qdot(i,3) = actuator_messages{i,1}.readings(9).state.joint_velocity;

    joint.RH.meas.qdot(i,1) = actuator_messages{i,1}.readings(10).state.joint_velocity;
    joint.RH.meas.qdot(i,2) = actuator_messages{i,1}.readings(11).state.joint_velocity;
    joint.RH.meas.qdot(i,3) = actuator_messages{i,1}.readings(12).state.joint_velocity;

    % Joint velocity desired
    joint.LF.des.qdot(i,1) = actuator_messages{i,1}.readings(1).commanded.velocity;
    joint.LF.des.qdot(i,2) = actuator_messages{i,1}.readings(2).commanded.velocity;
    joint.LF.des.qdot(i,3) = actuator_messages{i,1}.readings(3).commanded.velocity;
    
    joint.RF.des.qdot(i,1) = actuator_messages{i,1}.readings(4).commanded.velocity;
    joint.RF.des.qdot(i,2) = actuator_messages{i,1}.readings(5).commanded.velocity;
    joint.RF.des.qdot(i,3) = actuator_messages{i,1}.readings(6).commanded.velocity;
    
    joint.LH.des.qdot(i,1) = actuator_messages{i,1}.readings(7).commanded.velocity;
    joint.LH.des.qdot(i,2) = actuator_messages{i,1}.readings(8).commanded.velocity;
    joint.LH.des.qdot(i,3) = actuator_messages{i,1}.readings(9).commanded.velocity;

    joint.RH.des.qdot(i,1) = actuator_messages{i,1}.readings(10).commanded.velocity;
    joint.RH.des.qdot(i,2) = actuator_messages{i,1}.readings(11).commanded.velocity;
    joint.RH.des.qdot(i,3) = actuator_messages{i,1}.readings(12).commanded.velocity;
        
    % Joint torque measured
    joint.LF.meas.jointTorque(i,1) = actuator_messages{i,1}.readings(1).state.joint_torque;
    joint.LF.meas.jointTorque(i,2) = actuator_messages{i,1}.readings(2).state.joint_torque;
    joint.LF.meas.jointTorque(i,3) = actuator_messages{i,1}.readings(3).state.joint_torque;
    
    joint.RF.meas.jointTorque(i,1) = actuator_messages{i,1}.readings(4).state.joint_torque;
    joint.RF.meas.jointTorque(i,2) = actuator_messages{i,1}.readings(5).state.joint_torque;
    joint.RF.meas.jointTorque(i,3) = actuator_messages{i,1}.readings(6).state.joint_torque;
    
    joint.LH.meas.jointTorque(i,1) = actuator_messages{i,1}.readings(7).state.joint_torque;
    joint.LH.meas.jointTorque(i,2) = actuator_messages{i,1}.readings(8).state.joint_torque;
    joint.LH.meas.jointTorque(i,3) = actuator_messages{i,1}.readings(9).state.joint_torque;

    joint.RH.meas.jointTorque(i,1) = actuator_messages{i,1}.readings(10).state.joint_torque;
    joint.RH.meas.jointTorque(i,2) = actuator_messages{i,1}.readings(11).state.joint_torque;
    joint.RH.meas.jointTorque(i,3) = actuator_messages{i,1}.readings(12).state.joint_torque;    
    
    % Joint torque desired
    joint.LF.des.jointTorque(i,1) = actuator_messages{i,1}.readings(1).commanded.joint_torque;
    joint.LF.des.jointTorque(i,2) = actuator_messages{i,1}.readings(2).commanded.joint_torque;
    joint.LF.des.jointTorque(i,3) = actuator_messages{i,1}.readings(3).commanded.joint_torque;
    
    joint.RF.des.jointTorque(i,1) = actuator_messages{i,1}.readings(4).commanded.joint_torque;
    joint.RF.des.jointTorque(i,2) = actuator_messages{i,1}.readings(5).commanded.joint_torque;
    joint.RF.des.jointTorque(i,3) = actuator_messages{i,1}.readings(6).commanded.joint_torque;
    
    joint.LH.des.jointTorque(i,1) = actuator_messages{i,1}.readings(7).commanded.joint_torque;
    joint.LH.des.jointTorque(i,2) = actuator_messages{i,1}.readings(8).commanded.joint_torque;
    joint.LH.des.jointTorque(i,3) = actuator_messages{i,1}.readings(9).commanded.joint_torque;

    joint.RH.des.jointTorque(i,1) = actuator_messages{i,1}.readings(10).commanded.joint_torque;
    joint.RH.des.jointTorque(i,2) = actuator_messages{i,1}.readings(11).commanded.joint_torque;
    joint.RH.des.jointTorque(i,3) = actuator_messages{i,1}.readings(12).commanded.joint_torque;     
end

for i = 1:length(quadruped_state_messages)
    % Robot position (center of base?)
    base.time(i,1) = quadruped_state_messages{i,1}.header.stamp.time;
    base.position(i,:)    = quadruped_state_messages{i,1}.pose.pose.position;
    base.orientation(i,:) = quadruped_state_messages{i,1}.pose.pose.orientation;
end

joint.dt = diff(joint.time);
base.dt = diff(base.time);

base.vel = diff(base.position)./mean(base.dt);

base.time = base.time - base.time(1);
joint.time = joint.time - joint.time(1);

startTime = 2;
endTime = 7;

figure('name', 'base position')
subplot(3,1,1)
plot(base.time, base.position(:,1));
title('Base position x');
grid on
subplot(3,1,2)
plot(base.time, base.position(:,2));
title('Base position y');
grid on
subplot(3,1,3)
plot(base.time, base.position(:,3));
title('Base position z');
grid on

figure('name', 'joint position')
for i = 1:3
    hold on
    subplot(3,4,4*(i-1)+1)
    plot(joint.time, joint.LF.meas.q(:,i), joint.time, joint.LF.des.q(:,i))
    xlim([startTime, endTime]);
    grid on

    subplot(3,4,4*(i-1)+2)
    plot(joint.time, joint.RF.meas.q(:,i), joint.time, joint.RF.des.q(:,i))
    xlim([startTime, endTime]);
    grid on

    subplot(3,4,4*(i-1)+3)
    plot(joint.time, joint.LH.meas.q(:,i), joint.time, joint.LH.des.q(:,i))
    xlim([startTime, endTime]);
    grid on

    subplot(3,4,4*(i-1)+4)
    plot(joint.time, joint.RH.meas.q(:,i), joint.time, joint.RH.des.q(:,i))
    xlim([startTime, endTime]);
    grid on

    hold off
end

figure('name', 'joint velocity')
for i = 1:3
    hold on
    subplot(3,4,4*(i-1)+1)
    plot(joint.time, joint.LF.meas.qdot(:,i), joint.time, joint.LF.des.qdot(:,i))
    xlim([startTime, endTime]);
    grid on

    subplot(3,4,4*(i-1)+2)
    plot(joint.time, joint.RF.meas.qdot(:,i), joint.time, joint.RF.des.qdot(:,i))
    xlim([startTime, endTime]);
    grid on
    
    subplot(3,4,4*(i-1)+3)
    plot(joint.time, joint.LH.meas.qdot(:,i), joint.time, joint.LH.des.qdot(:,i))
    xlim([startTime, endTime]);
    grid on
    
    subplot(3,4,4*(i-1)+4)
    plot(joint.time, joint.RH.meas.qdot(:,i), joint.time, joint.RH.des.qdot(:,i))
    xlim([startTime, endTime]);
    grid on
    
    hold off
end

figure('name', 'joint torque')
for i = 1:3
    hold on
    subplot(3,4,4*(i-1)+1)
    plot(joint.time, joint.LF.meas.jointTorque(:,i), joint.time, joint.LF.des.jointTorque(:,i))
    xlim([startTime, endTime]);
    grid on
   
    subplot(3,4,4*(i-1)+2)
    plot(joint.time, joint.RF.meas.jointTorque(:,i), joint.time, joint.RF.des.jointTorque(:,i))
    xlim([startTime, endTime]);
    grid on
    
    subplot(3,4,4*(i-1)+3)
    plot(joint.time, joint.LH.meas.jointTorque(:,i), joint.time, joint.LH.des.jointTorque(:,i))
    xlim([startTime, endTime]);
    grid on
    
    subplot(3,4,4*(i-1)+4)
    plot(joint.time, joint.RH.meas.jointTorque(:,i), joint.time, joint.RH.des.jointTorque(:,i))
    xlim([startTime, endTime]);
    grid on
    
    hold off
end

ANYmalBearSlowTrotMeasured.joint = joint;
ANYmalBearSlowTrotMeasured.base = base;
save('ANYmalBearFastTrot2Measured.mat', '-struct', 'ANYmalBearSlowTrotMeasured');