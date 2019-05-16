%% getHipEECyclicMotion
% collects position of EE for each timestep from liftoff to next liftoff
% for a subset of the cycles when the motion is steady and averages the result
function [meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd] = getHipEECyclicData(tLiftoff, tTouchdown, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody)
%% Save position data for each cycle for each end effector

% determine minimum number of data points from liftoff to subsequent
% liftoff. The data for the cyclical motion will then be stored only up to
% this index number

for i = 1:length(tLiftoff(:,1))-2
    temp1 = length(relativeMotionHipEE.LF.position(floor(tLiftoff(i,1)/dt):floor(tLiftoff(i+1,1)/dt)));
    temp2 = length(relativeMotionHipEE.LH.position(floor(tLiftoff(i,2)/dt):floor(tLiftoff(i+1,2)/dt)));
    temp3 = length(relativeMotionHipEE.RF.position(floor(tLiftoff(i,3)/dt):floor(tLiftoff(i+1,3)/dt)));
    temp4 = length(relativeMotionHipEE.RH.position(floor(tLiftoff(i,4)/dt):floor(tLiftoff(i+1,4)/dt)));
    tempMin(i) = min([temp1, temp2, temp3, temp4]);
end
indexMax = min(tempMin);

%% save cyclic position of each end effector into an array. The 3rd dimension of the array is the step number

%LF
% use minStepCount-2 because some tasks had different sized arrays of data
% in the last couple steps likely due to violation of constraints in towr 

for i = 1:minStepCount-2
    temp.position = relativeMotionHipEE.LF.position(floor(tLiftoff(i,1)/dt):floor(tLiftoff(i+1,1)/dt),:);
    temp.velocity = relativeMotionHipEE.LF.velocity(floor(tLiftoff(i,1)/dt):floor(tLiftoff(i+1,1)/dt),:);
    temp.force = EE.LF.force(floor(tLiftoff(i,1)/dt):floor(tLiftoff(i+1,1)/dt),:);

    cyclicMotionHipEE.LF.position(:,:,i) = temp.position(1:indexMax,:);
    cyclicMotionHipEE.LF.velocity(:,:,i) = temp.velocity(1:indexMax,:);
    cyclicMotionHipEE.LF.force(:,:,i) = temp.force(1:indexMax,:);
end

%LH
for i = 1:minStepCount-2
    temp.position = relativeMotionHipEE.LH.position(floor(tLiftoff(i,2)/dt):floor(tLiftoff(i+1,2)/dt),:);
    temp.velocity = relativeMotionHipEE.LH.velocity(floor(tLiftoff(i,2)/dt):floor(tLiftoff(i+1,2)/dt),:);
    temp.force = EE.LH.force(floor(tLiftoff(i,2)/dt):floor(tLiftoff(i+1,2)/dt),:);

    cyclicMotionHipEE.LH.position(:,:,i) = temp.position(1:indexMax,:);
    cyclicMotionHipEE.LH.velocity(:,:,i) = temp.velocity(1:indexMax,:);
    cyclicMotionHipEE.LH.force(:,:,i) = temp.force(1:indexMax,:);
end

%RF
for i = 1:minStepCount-2
    temp.position = relativeMotionHipEE.RF.position(floor(tLiftoff(i,3)/dt):floor(tLiftoff(i+1,3)/dt),:);
    temp.velocity = relativeMotionHipEE.RF.velocity(floor(tLiftoff(i,3)/dt):floor(tLiftoff(i+1,3)/dt),:);
    temp.force = EE.RF.force(floor(tLiftoff(i,3)/dt):floor(tLiftoff(i+1,3)/dt),:);

    cyclicMotionHipEE.RF.position(:,:,i) = temp.position(1:indexMax,:);
    cyclicMotionHipEE.RF.velocity(:,:,i) = temp.velocity(1:indexMax,:);
    cyclicMotionHipEE.RF.force(:,:,i) = temp.force(1:indexMax,:);
end

%RH
for i = 1:minStepCount-2
    temp.position = relativeMotionHipEE.RH.position(floor(tLiftoff(i,4)/dt):floor(tLiftoff(i+1,4)/dt),:);
    temp.velocity = relativeMotionHipEE.RH.velocity(floor(tLiftoff(i,4)/dt):floor(tLiftoff(i+1,4)/dt),:);
    temp.force = EE.RH.force(floor(tLiftoff(i,4)/dt):floor(tLiftoff(i+1,4)/dt),:);

    cyclicMotionHipEE.RH.position(:,:,i) = temp.position(1:indexMax,:);
    cyclicMotionHipEE.RH.velocity(:,:,i) = temp.velocity(1:indexMax,:);
    cyclicMotionHipEE.RH.force(:,:,i) = temp.force(1:indexMax,:);
end

%% hip rotation from world frame
for i = 1:minStepCount-2
    temp.rotation = C_IBody(:,:,floor(tLiftoff(i,4)/dt):floor(tLiftoff(i+1,4)/dt));

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

meanCyclicMotionHipEE.LF.position = mean(cyclicMotionHipEE.LF.position(:,:,samplingStart:samplingEnd),3);
meanCyclicMotionHipEE.LH.position = mean(cyclicMotionHipEE.LH.position(:,:,samplingStart:samplingEnd),3);
meanCyclicMotionHipEE.RF.position = mean(cyclicMotionHipEE.RF.position(:,:,samplingStart:samplingEnd),3);
meanCyclicMotionHipEE.RH.position = mean(cyclicMotionHipEE.RH.position(:,:,samplingStart:samplingEnd),3);

meanCyclicMotionHipEE.LF.velocity = mean(cyclicMotionHipEE.LF.velocity(:,:,samplingStart:samplingEnd),3);
meanCyclicMotionHipEE.LH.velocity = mean(cyclicMotionHipEE.LH.velocity(:,:,samplingStart:samplingEnd),3);
meanCyclicMotionHipEE.RF.velocity = mean(cyclicMotionHipEE.RF.velocity(:,:,samplingStart:samplingEnd),3);
meanCyclicMotionHipEE.RH.velocity = mean(cyclicMotionHipEE.RH.velocity(:,:,samplingStart:samplingEnd),3);

meanCyclicMotionHipEE.LF.force = mean(cyclicMotionHipEE.LF.force(:,:,samplingStart:samplingEnd),3);
meanCyclicMotionHipEE.LH.force = mean(cyclicMotionHipEE.LH.force(:,:,samplingStart:samplingEnd),3);
meanCyclicMotionHipEE.RF.force = mean(cyclicMotionHipEE.RF.force(:,:,samplingStart:samplingEnd),3);
meanCyclicMotionHipEE.RH.force = mean(cyclicMotionHipEE.RH.force(:,:,samplingStart:samplingEnd),3);

% add new row at end of array with starting position to loop the position
% data
meanCyclicC_IBody(:,:,end+1) = meanCyclicC_IBody(:,:,1);

meanCyclicMotionHipEE.LF.position(end+1,:) = meanCyclicMotionHipEE.LF.position(1,:);
meanCyclicMotionHipEE.LH.position(end+1,:) = meanCyclicMotionHipEE.LH.position(1,:);
meanCyclicMotionHipEE.RF.position(end+1,:) = meanCyclicMotionHipEE.RF.position(1,:);
meanCyclicMotionHipEE.RH.position(end+1,:) = meanCyclicMotionHipEE.RH.position(1,:);

meanCyclicMotionHipEE.LF.velocity(end+1,:) = meanCyclicMotionHipEE.LF.velocity(1,:);
meanCyclicMotionHipEE.LH.velocity(end+1,:) = meanCyclicMotionHipEE.LH.velocity(1,:);
meanCyclicMotionHipEE.RF.velocity(end+1,:) = meanCyclicMotionHipEE.RF.velocity(1,:);
meanCyclicMotionHipEE.RH.velocity(end+1,:) = meanCyclicMotionHipEE.RH.velocity(1,:);

% set the final terms to zero indicating lift off of end effector
meanCyclicMotionHipEE.LF.force(end+1,:) = zeros(1,4);
meanCyclicMotionHipEE.LH.force(end+1,:) = zeros(1,4);
meanCyclicMotionHipEE.RF.force(end+1,:) = zeros(1,4);
meanCyclicMotionHipEE.RH.force(end+1,:) = zeros(1,4);