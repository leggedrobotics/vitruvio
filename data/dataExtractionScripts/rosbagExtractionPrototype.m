clear ros.Bag;
clear all
close all

% create a filename 'name.mat'
filename = 'nom_pushup_adjusting_foot_position_3.mat';
    
readROSbag = true;
if readROSbag
    %% Load a bag and get information about it
    % Using load() lets you auto-complete filepaths.
    %bag = ros.Bag.load('example.bag');
    bag = ros.Bag.load('nom_pushup_adjusting_foot_3.bag');
    bag.info()

    %% Read all messages on a few topics
    %topic1 = '/drawwire_sensor/drawwire_readings';
    topic2 = '/vitruvio_mover_interface/state';
    topic3 = '/rokubi_210_node/force_torque_sensor_measurements';

    %drawwire_messages = bag.readAll({topic1})';
    state_messages = bag.readAll({topic2})';
    force_sensor_messages = bag.readAll({topic3})';

    for i = 1:length(force_sensor_messages)
        experimentalData.forceSensor(i,:) = force_sensor_messages{i,1}.wrench.force;
        experimentalData.forceSensorTime(i,1) = force_sensor_messages{i,1}.header.stamp.time;
    end

    % for i = 1:length(drawwire_messages)
    %     experimentalData.drawwirePosition(i,1) = drawwire_messages{i,1}.position;
    %     experimentalData.drawwireVelocity(i,1) = drawwire_messages{i,1}.speed;
    %     experimentalData.drawwireTime(i,1)     = drawwire_messages{i,1}.header.stamp.time;
    % end
    % experimentalData.drawwireTime(:,1) = experimentalData.drawwireTime(:,1) - experimentalData.drawwireTime(1,1);


    % The first~10 values are not recorded, find the index when data recording
    % begins
    
    for i = 1:length(state_messages)
        if state_messages{i,1}.header.stamp.time ~= 0
            actuatorStartIndex = i;
            break
        end
    end

    for i = actuatorStartIndex:length(state_messages)
        experimentalData.position_HFE(i-(actuatorStartIndex-1),1) = state_messages{i,1}.state.position_HFE;
        experimentalData.position_KFE(i-(actuatorStartIndex-1),1) = state_messages{i,1}.state.position_KFE;
        experimentalData.velocity_HFE(i-(actuatorStartIndex-1),1) = state_messages{i,1}.state.velocity_HFE;
        experimentalData.velocity_KFE(i-(actuatorStartIndex-1),1) = state_messages{i,1}.state.velocity_KFE;
        experimentalData.current_HFE(i-(actuatorStartIndex-1),1)  = state_messages{i,1}.state.current_HFE/1000;
        experimentalData.current_KFE(i-(actuatorStartIndex-1),1)  = state_messages{i,1}.state.current_KFE/1000;
        experimentalData.time(i-(actuatorStartIndex-1),1)         = state_messages{i,1}.header.stamp.time;
    end
    experimentalData.time(:,1) = experimentalData.time(:,1) - experimentalData.time(1,1); 
    experimentalData.forceSensorTime(:,1) = experimentalData.forceSensorTime(:,1) - experimentalData.forceSensorTime(1,1); 

    save(filename, '-struct','experimentalData') 
else
    experimentalData = load(filename);
end

plotTimeLimits = [1 1000];

figure()

subplot(4,1,1)
hold on
title('Actuator position')
p(1) = plot(experimentalData.time(:,1), experimentalData.position_HFE(:,1), 'r', 'DisplayName','HFE');
p(2) = plot(experimentalData.time(:,1), experimentalData.position_KFE(:,1), 'b', 'DisplayName','KFE');
legend([p(1) p(2)])
xlim(plotTimeLimits);
xlabel('Time [s]')
ylabel('Actuator position [rad]')

subplot(4,1,2)
hold on
title('Actuator velocity')
p(1) = plot(experimentalData.time(:,1), experimentalData.velocity_HFE(:,1), 'r', 'DisplayName','HFE');
p(2) = plot(experimentalData.time(:,1), experimentalData.velocity_KFE(:,1), 'b', 'DisplayName','KFE');
legend([p(1) p(2)])
xlim(plotTimeLimits);
xlabel('Time [s]')
ylabel('Actuator velocity [rad/s]')

subplot(4,1,3)
hold on
title('Actuator current')
plot(experimentalData.time(:,1), experimentalData.current_HFE(:,1), 'r', 'DisplayName','HFE')
plot(experimentalData.time(:,1), experimentalData.current_KFE(:,1), 'b', 'DisplayName','KFE')
legend([p(1) p(2)])
xlim(plotTimeLimits);
xlabel('Time [s]')
ylabel('Actuator current [A]')
hold off

subplot(4,1,4)
title('Force sensor vertical force')
plot(experimentalData.forceSensorTime(:,1), experimentalData.forceSensor(:,3), 'r')
xlim(plotTimeLimits);
xlabel('Time [s]')
ylabel('Force Fz [N]')
hold off