% collects position of EE for each timestep from liftoff to next liftoff
% for a subset of the cycles when the motion is steady and averages the result

function [meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd, meanBasePosition] = getHipEECyclicData(tLiftoff, relativeMotionHipEE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody, EEnames, trajectoryData, legCount)
%% Get the average number of points in step number i
for i = 1:minStepCount-1 % because we consider future step, we can only count up until second last step
    % length of position vector from liftoff to subsequent liftoff (one
    % full step)
    temp1 = length(relativeMotionHipEE.LF.position(floor(tLiftoff.LF(i)/dt):floor(tLiftoff.LF(i+1)/dt)));
    temp2 = []; temp3 = []; temp4 = []; % Initialize other legs to be empty. If these legs exist, the values will be filled in.
    if legCount > 1
        temp2 = length(relativeMotionHipEE.RF.position(floor(tLiftoff.RF(i)/dt):floor(tLiftoff.RF(i+1)/dt)));
    end
    if legCount > 2
        temp3 = length(relativeMotionHipEE.LH.position(floor(tLiftoff.LH(i)/dt):floor(tLiftoff.LH(i+1)/dt)));
    end
    if legCount > 3
        temp4 = length(relativeMotionHipEE.RH.position(floor(tLiftoff.RH(i)/dt):floor(tLiftoff.RH(i+1)/dt)));
    end 
    indexCount(i) = mean([temp1, temp2, temp3, temp4]);
end
meanIndexCount = round(mean(indexCount)); % mean number of points (time) in one cycle of the leg

%% save cyclic pos, vel, force of each end effector into an array. 
% The 3rd dimension of the array is the step number.
for i = 1:legCount
    EEselection = EEnames(i,:);
    for i = 1:minStepCount-2 % because we sample to next step + 20% we only count up until the final step -2
        % Save position elements from liftoff through to +120% of a
        % cycle. This helps us to get a more smooth average motion as the
        % duration for one step can vary slightly.
        temp.position = relativeMotionHipEE.(EEselection).position(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i)/dt)+floor(1.2*meanIndexCount),:);
        temp.force = trajectoryData.(EEselection).force(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i)/dt)+floor(1.2*meanIndexCount),:);

        cyclicMotionHipEE.(EEselection).position(:,:,i) = temp.position;
        cyclicMotionHipEE.(EEselection).force(:,:,i) = temp.force;
    end
end

%% body rotation and position in inertial frame
for i = 1:minStepCount-2 
    for j = 1:legCount
        EEselection = EEnames(j,:);
        % by saving the body rotation separately for each leg we can synchronize
        % the body rotation and leg motion
        cyclicC_IBody.(EEselection)(:,:,:,i) = C_IBody(:,:,floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i)/dt)+floor(1.2*meanIndexCount));
        basePosition.(EEselection)(:,:,i) = trajectoryData.base.position(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i)/dt)+floor(1.2*meanIndexCount),:); 
    end
end

%% Average EE position and base rotation at corresponding times for each cycle

% The steps from samplingStart to samplingEnd are used in the average
% calculation
samplingStart = round(removalRatioStart*minStepCount);
% minimum starting index is 1
samplingStart(samplingStart<1) = 1;
samplingEnd = round((1-removalRatioEnd)*minStepCount-1);
% maximum ending index is number of steps captured in cyclicMotionHipEE
samplingEnd(samplingEnd>length(cyclicMotionHipEE.(EEselection).position(1,1,:))) = length(cyclicMotionHipEE.(EEselection).position(1,1,:));

% average of corresponding points in each cycle
for i = 1:legCount
    EEselection = EEnames(i,:);
    
    % EE position and force
    meanCyclicMotionHipEE.(EEselection).position = mean(cyclicMotionHipEE.(EEselection).position(:,:,samplingStart:samplingEnd),3);
    meanCyclicMotionHipEE.(EEselection).force    = mean(cyclicMotionHipEE.(EEselection).force(:,:,samplingStart:samplingEnd),3);    
    
    % average rotation matrix of body in inertia frametemp3
    cyclicC_IBody.(EEselection) = cyclicC_IBody.(EEselection)(:,:,:,samplingStart:samplingEnd);
    meanCyclicC_IBody.(EEselection) = mean(cyclicC_IBody.(EEselection),4);
    
    % average rotation matrix of body in inertia frame    
    meanBasePosition.(EEselection) = mean(basePosition.(EEselection)(:,:,samplingStart:samplingEnd),3);
end

%% Find point that completes cycle loop
% Find point with minimum distance to the first point. This is where the
% step cycle loops. Save the data up to this index+2 to preserve the full
% motion when taking finite differences

for i = 1:legCount
    EEselection = EEnames(i,:);
    % vector from current position to position at first timestep
    offsetFromLiftoffPosition.(EEselection) = meanCyclicMotionHipEE.(EEselection).position(1,:) - meanCyclicMotionHipEE.(EEselection).position;
   
    for j = 1:length(meanCyclicMotionHipEE.(EEselection).position)
        % Euclidean norm of the distance from first position at each
        % timestep
        offsetMagnitude(j,1) = norm(offsetFromLiftoffPosition.(EEselection)(j,:));
    end
    % Look for the minimum offset to the first position. Only look at the offset data for the second half of the motion to
    % avoid finding the minimum offset as the point at liftoff+1.
    for j = 1:round(0.5*length(meanCyclicMotionHipEE.(EEselection).position))
        offsetMagnitude(j,1) = 1; % Force the min offset to be found in the second half of the motion by setting the offset in the first half to be arbitrarily large.
    end
        [minValue.(EEselection), indexMin.(EEselection)] = min(offsetMagnitude); 
        indexMin.(EEselection) = indexMin.(EEselection) - 1; % save data up to point before liftoff so we can complete the loop exactly with the first liftoff position

         %% Save the data only up to this point which is one point before completion of the loop. 
         % Then artificially complete the loop by extending the array to
         % end with the same data as it started
    
         % Position data
         % After completing the loop, add two more points to position so we have a full
         % loop once we take second finite difference for acceleration.
         meanCyclicMotionHipEE.(EEselection).position          = meanCyclicMotionHipEE.(EEselection).position(1:indexMin.(EEselection),:);
         meanCyclicMotionHipEE.(EEselection).position(end+1,:) = meanCyclicMotionHipEE.(EEselection).position(1,:); % completes the loop
         meanCyclicMotionHipEE.(EEselection).position(end+1,:) = meanCyclicMotionHipEE.(EEselection).position(2,:);
         meanCyclicMotionHipEE.(EEselection).position(end+1,:) = meanCyclicMotionHipEE.(EEselection).position(3,:);
         
         % Force data        
         meanCyclicMotionHipEE.(EEselection).force          = meanCyclicMotionHipEE.(EEselection).force(1:indexMin.(EEselection),:);
         meanCyclicMotionHipEE.(EEselection).force(end+1,:) = meanCyclicMotionHipEE.(EEselection).force(1,:);
         meanCyclicMotionHipEE.(EEselection).force(end+1,:) = meanCyclicMotionHipEE.(EEselection).force(2,:);
         meanCyclicMotionHipEE.(EEselection).force(end+1,:) = meanCyclicMotionHipEE.(EEselection).force(3,:);

         % Body rotation data
         meanEuler = rotm2eul(meanCyclicC_IBody.(EEselection), 'ZYX');
         meanCyclicMotionHipEE.body.eulerAngles.(EEselection) = meanEuler;
         
         % Body position data
         meanBasePosition.(EEselection)          = meanBasePosition.(EEselection)(1:indexMin.(EEselection),:);
         meanBasePosition.(EEselection)(end+1,:) = meanBasePosition.(EEselection)(1,:);
end