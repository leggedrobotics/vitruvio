function [interpolatedPositionData, interpolatedVelocityData, interpolatedForceData, interpolatedBodyRotationData, newTimeData] = generateAdditionalPoints(Leg, relativeMotionHipEE, EE, C_IBody, EEnames, i) 
    % insert row of NaN where a point should be interpolated and concatenate with the old array.
    for j = 1:4
        EEselection = EEnames(j,:);
        
        tempPosition.(EEselection) = [relativeMotionHipEE.(EEselection).position(1:i,:);      ...
                                      nan(1,3);                                               ...
                                      relativeMotionHipEE.(EEselection).position(i+1:end,:)];

        tempVelocity.(EEselection) = [relativeMotionHipEE.(EEselection).velocity(1:i,:);      ...
                                      nan(1,3);                                               ...
                                      relativeMotionHipEE.(EEselection).velocity(i+1:end,:)];

        tempForce.(EEselection) = [EE.(EEselection).force(1:i,:);      ...
                                   nan(1,4);                           ...
                                   EE.(EEselection).force(i+1:end,:)];   
                               
        interpolatedPositionData.(EEselection)     = fillmissing(tempPosition.(EEselection), 'linear');
        interpolatedVelocityData.(EEselection)     = fillmissing(tempVelocity.(EEselection), 'linear');
        interpolatedForceData.(EEselection)        = fillmissing(tempForce.(EEselection), 'linear');
    end
    
    tempBodyRotation = cat(3,C_IBody(:,:,1:i),nan(3,3),C_IBody(:,:,i+1:end));
    interpolatedBodyRotationData = fillmissing(tempBodyRotation, 'linear', 3);
    
    newTimeData = [Leg.time(1:i);                     ...
                   0.5*(Leg.time(i) + Leg.time(i+1)); ...
                   Leg.time(i+1:end)];
end