function [interpolatedPositionData, interpolatedVelocityData, interpolatedForceData, interpolatedBodyRotationData, interpolatedBasePosition, interpolatedInertialFrameEEPosition] = generateAdditionalPoints(relativeMotionHipEE, trajectoryData, C_IBody, EEselection, Leg)
 
    tempPosition = nan(2*length(relativeMotionHipEE.(EEselection).position)-1, 3);
    tempVelocity = nan(2*length(relativeMotionHipEE.(EEselection).velocity)-1, 3);   
    tempForce = nan(2*length(trajectoryData.(EEselection).force)-1, 4);
    tempBodyRotation = nan(3,3,2*length(C_IBody(1,1,:))-1);
    tempBasePosition = nan(2*length(trajectoryData.base.position)-1, 3);     
    tempEEPosition = nan(2*length(Leg.inertialFrame.EEpositionTrimmed.(EEselection)-1), 3);

    for i = 1:length(relativeMotionHipEE.(EEselection).position)
        tempPosition(2*i-1,:) = relativeMotionHipEE.(EEselection).position(i,:);
    end
    
    for i = 1:length(relativeMotionHipEE.(EEselection).velocity)
        tempVelocity(2*i-1,:) = relativeMotionHipEE.(EEselection).velocity(i,:);
    end
    
    for i = 1:length(trajectoryData.(EEselection).force)
        tempForce(2*i-1,:) = trajectoryData.(EEselection).force(i,:);
    end
    
    for i = 1:length(C_IBody(1,1,:))
        tempBodyRotation(:,:,2*i-1) = C_IBody(:,:,i);
    end
    
    for i = 1:length(trajectoryData.base.position)
        tempBasePosition(2*i-1,:) = trajectoryData.base.position(i,:);
    end
    
    for i = 1:length(Leg.inertialFrame.EEpositionTrimmed.(EEselection))
        tempEEPosition(2*i-1,:) = Leg.inertialFrame.EEpositionTrimmed.(EEselection)(i,:);
    end

    interpolatedPositionData     = fillmissing(tempPosition, 'spline');
    interpolatedVelocityData     = fillmissing(tempVelocity, 'linear');
    interpolatedForceData        = fillmissing(tempForce, 'linear');
    interpolatedBodyRotationData = fillmissing(tempBodyRotation, 'linear', 3);
    interpolatedBasePosition     = fillmissing(tempBasePosition, 'linear');
    interpolatedInertialFrameEEPosition = fillmissing(tempEEPosition, 'linear');
end