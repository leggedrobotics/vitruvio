% Takes the relative motion data and force data as input and returns the
% data after cropping by the start and end removal ratios. The data is not
% averaged or made cyclical.
function [meanCyclicMotionHipEE, meanCyclicC_IBody, basePosition] = getHipEEFullMotionData(relativeMotionHipEE, removalRatioStart, removalRatioEnd, C_IBody, EEnames, trajectoryData, legCount)   
    for i = 1:legCount
        EEselection = EEnames(i,:);

        % Rather than averaging multiple steps into one step, get the relative motion of the EE wrt hip over multiple steps.
        startIndex = round(length(relativeMotionHipEE.(EEselection).position)*removalRatioStart);
        startIndex(startIndex<1) = 1;
        endIndex = round(length(relativeMotionHipEE.(EEselection).position)*(1-removalRatioEnd));
       
        meanCyclicC_IBody.(EEselection) = C_IBody(:,:,startIndex:endIndex);
      
        meanCyclicMotionHipEE.(EEselection).position = relativeMotionHipEE.(EEselection).position(startIndex:endIndex,:);
        meanCyclicMotionHipEE.(EEselection).velocity = relativeMotionHipEE.(EEselection).velocity(startIndex:endIndex-1,:);
        meanCyclicMotionHipEE.(EEselection).force    = trajectoryData.(EEselection).force(startIndex:endIndex,:);
       
        meanEuler = rotm2eul(C_IBody(:,:,startIndex:endIndex), 'ZYX');
        meanCyclicMotionHipEE.body.eulerAngles.(EEselection) = meanEuler;
        basePosition.(EEselection) = trajectoryData.base.position(startIndex:endIndex,:);  % here we save the same data for each EEselection to generalize visualization
    end     
end