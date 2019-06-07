% %% getHipEECyclicMotion
% % collects position of EE for each timestep from liftoff to next liftoff
% % for a subset of the cycles when the motion is steady and averages the result
% function [meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd, meanTouchdownIndex] = getHipEECyclicData(quadruped, tLiftoff, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody, EEnames)
% %% Save position data for each cycle for each end effector
% % determine minimum number of data points from liftoff to subsequent
% % liftoff. The data for the cyclical motion will then be stored only up to
% % this index number
% for i = 1:length(tLiftoff.LF)-2
%     temp1 = length(relativeMotionHipEE.LF.position(floor(tLiftoff.LF(i)/dt):floor(tLiftoff.LF(i+1)/dt)));
%     temp2 = length(relativeMotionHipEE.LH.position(floor(tLiftoff.LH(i)/dt):floor(tLiftoff.LH(i+1)/dt)));
%     temp3 = length(relativeMotionHipEE.RF.position(floor(tLiftoff.RF(i)/dt):floor(tLiftoff.RF(i+1)/dt)));
%     temp4 = length(relativeMotionHipEE.RH.position(floor(tLiftoff.RH(i)/dt):floor(tLiftoff.RH(i+1)/dt)));
%     tempMin(i) = min([temp1, temp2, temp3, temp4]);
% end
% indexMax = min(tempMin);
% 
% %% save cyclic position of each end effector into an array. The 3rd dimension of the array is the step number
% % use minStepCount-2 because some tasks had different sized arrays of data
% % in the last couple steps likely due to violation of constraints in towr
% % this way we can neglect those last couple steps
% for i = 1:4
%     EEselection = EEnames(i,:);
%     for i = 1:minStepCount-2
%         temp.position = relativeMotionHipEE.(EEselection).position(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i+1)/dt),:);
%         temp.velocity = relativeMotionHipEE.(EEselection).velocity(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i+1)/dt),:);
%         temp.force = EE.(EEselection).force(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i+1)/dt),:);
% 
%         cyclicMotionHipEE.(EEselection).position(:,:,i) = temp.position(1:indexMax,:);
%         cyclicMotionHipEE.(EEselection).velocity(:,:,i) = temp.velocity(1:indexMax,:);
%         cyclicMotionHipEE.(EEselection).force(:,:,i) = temp.force(1:indexMax,:);
%     end
% end
% 
% %% hip rotation from world frame
% for i = 1:minStepCount-2
%     temp.rotation = C_IBody(:,:,floor(tLiftoff.LF(i)/dt):floor(tLiftoff.LF(i+1)/dt));
%     cyclicC_IBody(:,:,:,i) = temp.rotation(:,:,1:indexMax);
% end
% 
% %% Average the position of the end effector at corresponding times for each cycle
% % only consider cycles outside of the acceleration and deceleration phases
% 
% samplingStart = floor(removalRatioStart*(minStepCount-1));
% % minimum starting index is 1
% if samplingStart == 0
%     samplingStart = 1;
% end
% samplingEnd = round((1-removalRatioEnd)*(minStepCount-1));
% 
% meanCyclicC_IBody = mean(cyclicC_IBody(:,:,:,samplingStart:samplingEnd),4);
% 
% % average of corresponding points in each cycle
% for i = 1:4
%     EEselection = EEnames(i,:);
%     meanCyclicMotionHipEE.(EEselection).position = mean(cyclicMotionHipEE.(EEselection).position(:,:,samplingStart:samplingEnd),3);
%     meanCyclicMotionHipEE.(EEselection).velocity = mean(cyclicMotionHipEE.(EEselection).velocity(:,:,samplingStart:samplingEnd),3);
%     meanCyclicMotionHipEE.(EEselection).force = mean(cyclicMotionHipEE.(EEselection).force(:,:,samplingStart:samplingEnd),3);
%     % add new row at end of array with starting position to loop the position,
%     % velocity and force data
%     meanCyclicMotionHipEE.(EEselection).position(end+1,:) = meanCyclicMotionHipEE.(EEselection).position(1,:);
%     meanCyclicMotionHipEE.(EEselection).velocity(end+1,:) = meanCyclicMotionHipEE.(EEselection).velocity(1,:);
%     meanCyclicMotionHipEE.(EEselection).force(end+1,:) = meanCyclicMotionHipEE.(EEselection).force(1,:);
% end
% meanCyclicC_IBody(:,:,end+1) = meanCyclicC_IBody(:,:,1);
% 
% % mean touchdown index (used for graphing). The liftoff always is index 1
% for j = 1:4
%     EEselection = EEnames(j,:);
%     for i=2:length(meanCyclicMotionHipEE.(EEselection).force(:,3))
%         if(meanCyclicMotionHipEE.(EEselection).force(i-1,3) ==0 ) && (meanCyclicMotionHipEE.(EEselection).force(i,3) ~= 0)
%             meanTouchdownIndex.(EEselection) = i;
%         end
%     end
% end
% 
% % mean cyclic euler angles for body rotation
% meanEuler = rotm2eul(meanCyclicC_IBody, 'ZYX');
% meanCyclicMotionHipEE.body.eulerAngles = meanEuler;
% 


%% getHipEECyclicMotion
% collects position of EE for each timestep from liftoff to next liftoff
% for a subset of the cycles when the motion is steady and averages the result
function [meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd, meanTouchdownIndex] = getHipEECyclicData(quadruped, tLiftoff, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody, EEnames)
%% Save position data for each cycle for each end effector
% determine minimum number of data points from liftoff to subsequent
% liftoff. The data for the cyclical motion will then be stored only up to
% this index number
for i = 1:length(tLiftoff.LF)-2
    temp1 = length(relativeMotionHipEE.LF.position(floor(tLiftoff.LF(i)/dt):floor(tLiftoff.LF(i+1)/dt)));
    temp2 = length(relativeMotionHipEE.LH.position(floor(tLiftoff.LH(i)/dt):floor(tLiftoff.LH(i+1)/dt)));
    temp3 = length(relativeMotionHipEE.RF.position(floor(tLiftoff.RF(i)/dt):floor(tLiftoff.RF(i+1)/dt)));
    temp4 = length(relativeMotionHipEE.RH.position(floor(tLiftoff.RH(i)/dt):floor(tLiftoff.RH(i+1)/dt)));
    tempMin(i) = min([temp1, temp2, temp3, temp4]);
end
indexMax = min(tempMin);

%% save cyclic position of each end effector into an array. The 3rd dimension of the array is the step number
% use minStepCount-2 because some tasks had different sized arrays of data
% in the last couple steps likely due to violation of constraints in towr
% this way we can neglect those last couple steps
for i = 1:4
    EEselection = EEnames(i,:);
    for i = 1:minStepCount-2
        temp.position = relativeMotionHipEE.(EEselection).position(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i+1)/dt),:);
        temp.velocity = relativeMotionHipEE.(EEselection).velocity(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i+1)/dt),:);
        temp.force = EE.(EEselection).force(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i+1)/dt),:);

        cyclicMotionHipEE.(EEselection).position(:,:,i) = temp.position(1:indexMax,:);
        cyclicMotionHipEE.(EEselection).velocity(:,:,i) = temp.velocity(1:indexMax,:);
        cyclicMotionHipEE.(EEselection).force(:,:,i) = temp.force(1:indexMax,:);
    end
end

%% hip rotation from world frame
for i = 1:minStepCount-2
    temp.rotation = C_IBody(:,:,floor(tLiftoff.LF(i)/dt):floor(tLiftoff.LF(i+1)/dt));
    cyclicC_IBody(:,:,:,i) = temp.rotation(:,:,1:indexMax);
end
%% Average the position of the end effector at corresponding times for each cycle
% only consider cycles outside of the acceleration and deceleration phases

samplingStart = floor(removalRatioStart*(minStepCount-1));
% minimum starting index is 1
if samplingStart == 0
    samplingStart = 1;
end
samplingEnd = round((1-removalRatioEnd)*(minStepCount-1));

meanCyclicC_IBody = mean(cyclicC_IBody(:,:,:,samplingStart:samplingEnd),4);

% average of corresponding points in each cycle
for i = 1:4
    EEselection = EEnames(i,:);
    meanCyclicMotionHipEE.(EEselection).position = mean(cyclicMotionHipEE.(EEselection).position(:,:,samplingStart:samplingEnd),3);
    meanCyclicMotionHipEE.(EEselection).velocity = mean(cyclicMotionHipEE.(EEselection).velocity(:,:,samplingStart:samplingEnd),3);
    meanCyclicMotionHipEE.(EEselection).force = mean(cyclicMotionHipEE.(EEselection).force(:,:,samplingStart:samplingEnd),3);
    % add new row at end of array with starting position to loop the position,
    % velocity and force data. Add two terms to position data so when we
    % take finite differences later we still have full loop for torque
    meanCyclicMotionHipEE.(EEselection).position(end+1,:) = meanCyclicMotionHipEE.(EEselection).position(1,:);   
    meanCyclicMotionHipEE.(EEselection).position(end+1,:) = meanCyclicMotionHipEE.(EEselection).position(2,:);   
    meanCyclicMotionHipEE.(EEselection).position(end+1,:) = meanCyclicMotionHipEE.(EEselection).position(3,:);   

    meanCyclicMotionHipEE.(EEselection).velocity(end+1,:) = meanCyclicMotionHipEE.(EEselection).velocity(1,:);
    meanCyclicMotionHipEE.(EEselection).force(end+1,:) = meanCyclicMotionHipEE.(EEselection).force(1,:);
end
meanCyclicC_IBody(:,:,end+1) = meanCyclicC_IBody(:,:,1);

% mean touchdown index (used for graphing). The liftoff always is index 1
for j = 1:4
    EEselection = EEnames(j,:);
    for i=2:length(meanCyclicMotionHipEE.(EEselection).force(:,3))
        if(meanCyclicMotionHipEE.(EEselection).force(i-1,3) ==0 ) && (meanCyclicMotionHipEE.(EEselection).force(i,3) ~= 0)
            meanTouchdownIndex.(EEselection) = i;
        end
    end
end

% mean cyclic euler angles for body rotation
meanEuler = rotm2eul(meanCyclicC_IBody, 'ZYX');
meanCyclicMotionHipEE.body.eulerAngles = meanEuler;
meanCyclicMotionHipEE.body.eulerAngles(end+1,:) = meanCyclicMotionHipEE.body.eulerAngles(1,:);
meanCyclicMotionHipEE.body.eulerAngles(end+1,:) = meanCyclicMotionHipEE.body.eulerAngles(2,:);
meanCyclicMotionHipEE.body.eulerAngles(end+1,:) = meanCyclicMotionHipEE.body.eulerAngles(3,:);