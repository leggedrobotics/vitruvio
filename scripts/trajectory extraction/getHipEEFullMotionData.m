function [meanCyclicMotionHipEE, meanCyclicC_IBody, basePosition] = getHipEEFullMotionData(dataExtraction, tLiftoff, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody, EEnames, base);
             
    for i = 1:4
        EEselection = EEnames(i,:);

         % Rather than averaging multiple steps, get the relative motion of the EE wrt hip over multiple steps.
        startIndex = round(length(relativeMotionHipEE.(EEselection).position)*removalRatioStart);
        startIndex(startIndex<1) = 1;
        endIndex = round(length(relativeMotionHipEE.(EEselection).position)*(1-removalRatioEnd));
       
        meanCyclicC_IBody.(EEselection) = C_IBody(:,:,startIndex:endIndex);
      
        meanCyclicMotionHipEE.(EEselection).position = relativeMotionHipEE.(EEselection).position(startIndex:endIndex,:);
        meanCyclicMotionHipEE.(EEselection).velocity = relativeMotionHipEE.(EEselection).velocity(startIndex:endIndex-1,:);
        meanCyclicMotionHipEE.(EEselection).force = EE.(EEselection).force(startIndex:endIndex,:);
       
        meanEuler = rotm2eul(C_IBody(:,:,startIndex:endIndex), 'ZYX');
        meanCyclicMotionHipEE.body.eulerAngles.(EEselection) = meanEuler;
        basePosition.(EEselection) = base.position(startIndex:endIndex,:);  % here we save the same data for each EEselection to generalize visualization
    end  
    
end