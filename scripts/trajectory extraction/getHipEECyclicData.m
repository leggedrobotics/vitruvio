% collects position of EE for each timestep from liftoff to next liftoff
% for a subset of the cycles when the motion is steady and averages the result

function [meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd, meanTouchdownIndex, basePosition] = getHipEECyclicData(dataExtraction, tLiftoff, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody, EEnames, base)
%% Get the average number of points in one cycle of the leg
for i = 1:length(tLiftoff.LF)-2
    temp1 = length(relativeMotionHipEE.LF.position(floor(tLiftoff.LF(i)/dt):floor(tLiftoff.LF(i+1)/dt)));
    temp2 = length(relativeMotionHipEE.LH.position(floor(tLiftoff.LH(i)/dt):floor(tLiftoff.LH(i+1)/dt)));
    temp3 = length(relativeMotionHipEE.RF.position(floor(tLiftoff.RF(i)/dt):floor(tLiftoff.RF(i+1)/dt)));
    temp4 = length(relativeMotionHipEE.RH.position(floor(tLiftoff.RH(i)/dt):floor(tLiftoff.RH(i+1)/dt)));
    indexCount(i) = mean([temp1, temp2, temp3, temp4]);
end
meanIndexCount = round(mean(indexCount)); % mean number of points in one cycle of the leg

%% save cyclic pos, vel, force of each end effector into an array. 
% The 3rd dimension of the array is the step number.
% use minStepCount-1 to neglect the last steps of the cycle which are more prone
% to constraint violation in towr
for i = 1:4
    EEselection = EEnames(i,:);
%     for i = 1:minStepCount-1
    for i = 1:minStepCount-1
        % Save position elements from liftoff to next liftoff + 20% of a
        % cycle. This helps us to get a more smooth average motion as some
        % steps have different durations
        temp.position = relativeMotionHipEE.(EEselection).position(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i)/dt)+floor(1.2*meanIndexCount),:);
        temp.velocity = relativeMotionHipEE.(EEselection).velocity(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i)/dt)+floor(1.2*meanIndexCount),:);
        temp.force = EE.(EEselection).force(floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i)/dt)+floor(1.2*meanIndexCount),:);

        cyclicMotionHipEE.(EEselection).position(:,:,i) = temp.position;
        cyclicMotionHipEE.(EEselection).velocity(:,:,i) = temp.velocity;
        cyclicMotionHipEE.(EEselection).force(:,:,i) = temp.force;
    end
end

%% body rotation in inertial frame
for i = 1:minStepCount-1
    for j = 1:4
        EEselection = EEnames(j,:);
        % by saving the body rotation separately for each leg we can synchronize
        % the body rotation and leg motion
        cyclicC_IBody.(EEselection)(:,:,:,i) = C_IBody(:,:,floor(tLiftoff.(EEselection)(i)/dt):floor(tLiftoff.(EEselection)(i)/dt)+floor(1.2*meanIndexCount));
    end
end

%% Average EE position and base rotation at corresponding times for each cycle

% The steps from samplingStart to samplingEnd are used in the average
% calculation
samplingStart = round(removalRatioStart*(minStepCount-1));
if samplingStart == 0
    samplingStart = 1; % minimum starting index is 1
end
samplingEnd = round((1-removalRatioEnd)*(minStepCount-1));

% average of corresponding points in each cycle
for i = 1:4
    EEselection = EEnames(i,:);
    meanCyclicMotionHipEE.(EEselection).position = mean(cyclicMotionHipEE.(EEselection).position(:,:,samplingStart:samplingEnd),3);
    meanCyclicMotionHipEE.(EEselection).velocity = mean(cyclicMotionHipEE.(EEselection).velocity(:,:,samplingStart:samplingEnd),3);
    meanCyclicMotionHipEE.(EEselection).force = mean(cyclicMotionHipEE.(EEselection).force(:,:,samplingStart:samplingEnd),3);    
    % average rotation matrix of body in inertia frame
    cyclicC_IBody.(EEselection) = cyclicC_IBody.(EEselection)(:,:,:,samplingStart:samplingEnd);
    meanCyclicC_IBody.(EEselection) = mean(cyclicC_IBody.(EEselection),4);
end

%% Find point that completes cycle loop
% Find point with minimum distance to the first point. This is where the
% step cycle loops. Save the data up to this index+2 to preserve the full
% motion when taking finite differences
if dataExtraction.averageStepsForCyclicalMotion
for i = 1:4
    EEselection = EEnames(i,:);
    offsetFromLiftoffPosition.(EEselection) = meanCyclicMotionHipEE.(EEselection).position(1,:) - meanCyclicMotionHipEE.(EEselection).position(:,:);
   
    for j = 1:length(meanCyclicMotionHipEE.(EEselection).position)
        offsetMagnitude(j,1) = norm(offsetFromLiftoffPosition.(EEselection)(j,:));
    end
    % only look at the offset data for the second half of the motion to
    % avoid finding the minimum offset as the point at liftoff+1.
    for j = 1:round(0.5*length(meanCyclicMotionHipEE.(EEselection).position))
        offsetMagnitude(j,1) = 1; % forces the min offset to be found in the second half of the motion
    end
        [minValue.(EEselection), indexMin.(EEselection)] = min(offsetMagnitude);
        indexMin.(EEselection) = indexMin.(EEselection) - 1; % save data up to point before liftoff so we can complete the loop exactly with the first liftoff position

         meanCyclicMotionHipEE.(EEselection).position = meanCyclicMotionHipEE.(EEselection).position(1:indexMin.(EEselection),:);
         meanCyclicMotionHipEE.(EEselection).position(end+1,:) = meanCyclicMotionHipEE.(EEselection).position(1,:); % completes the loop
         
         % after completing the loop, add two more points so we have a full
         % loop once we take second finite difference for acceleration.
         meanCyclicMotionHipEE.(EEselection).position(end+1,:) = meanCyclicMotionHipEE.(EEselection).position(2,:);
         meanCyclicMotionHipEE.(EEselection).position(end+1,:) = meanCyclicMotionHipEE.(EEselection).position(3,:);
         
         meanCyclicMotionHipEE.(EEselection).velocity          = meanCyclicMotionHipEE.(EEselection).velocity(1:indexMin.(EEselection),:);
         meanCyclicMotionHipEE.(EEselection).velocity(end+1,:) = meanCyclicMotionHipEE.(EEselection).velocity(1,:);

         meanCyclicMotionHipEE.(EEselection).force          = meanCyclicMotionHipEE.(EEselection).force(1:indexMin.(EEselection),:);
         meanCyclicMotionHipEE.(EEselection).force(end+1,:) = meanCyclicMotionHipEE.(EEselection).force(1,:);
         meanCyclicMotionHipEE.(EEselection).force(end+1,:) = meanCyclicMotionHipEE.(EEselection).force(2,:);
         meanCyclicMotionHipEE.(EEselection).force(end+1,:) = meanCyclicMotionHipEE.(EEselection).force(3,:);

         % mean cyclic euler angles for body rotation
         % meanEuler = rotm2eul(meanCyclicC_IBody, 'ZYX');
         meanEuler = rotm2eul(meanCyclicC_IBody.(EEselection), 'ZYX');
         meanCyclicMotionHipEE.body.eulerAngles.(EEselection) = meanEuler;
end

%% Rather than averaging multiple steps, get the relative motion of the EE wrt hip over multiple steps.
elseif ~ dataExtraction.averageStepsForCyclicalMotion
    startIndex = round(length(relativeMotionHipEE.(EEselection).position)*removalRatioStart);
    endIndex = round(length(relativeMotionHipEE.(EEselection).position)*(1-removalRatioEnd));
    meanEuler = rotm2eul(C_IBody(:,:,startIndex:endIndex), 'ZYX');
    basePosition = base.position(startIndex:endIndex,:);    
    for i = 1:4
        EEselection = EEnames(i,:);
        meanCyclicMotionHipEE.(EEselection).position = relativeMotionHipEE.(EEselection).position(startIndex:endIndex,:);
        meanCyclicMotionHipEE.(EEselection).velocity = relativeMotionHipEE.(EEselection).velocity(startIndex:endIndex,:);
        meanCyclicMotionHipEE.(EEselection).force = EE.(EEselection).force(startIndex:endIndex,:);
        meanCyclicMotionHipEE.body.eulerAngles.(EEselection) = meanEuler;
    end
end

%% mean touchdown index (used for graphing)
% The liftoff always is index 1
for j = 1:4
    EEselection = EEnames(j,:);
    for i=2:length(meanCyclicMotionHipEE.(EEselection).force(:,3))
        if(meanCyclicMotionHipEE.(EEselection).force(i-1,3) ==0 ) && (meanCyclicMotionHipEE.(EEselection).force(i,3) ~= 0)
            meanTouchdownIndex.(EEselection) = i;
        end
    end
end