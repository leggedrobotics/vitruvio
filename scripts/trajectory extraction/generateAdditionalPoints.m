function [interpolatedPositionData, interpolatedVelocityData, interpolatedForceData, interpolatedBodyRotationData] = generateAdditionalPoints(relativeMotionHipEE, EE, C_IBody, EEselection)
 
    tempPosition = nan(2*length(relativeMotionHipEE.(EEselection).position),3);
    tempVelocity = nan(2*length(relativeMotionHipEE.(EEselection).velocity),3);   
    tempForce = nan(2*length(EE.(EEselection).force),4);
    tempBodyRotation = nan(3,3,2*length(C_IBody(1,1,:)));

for i = 1:length(relativeMotionHipEE.(EEselection).position)
    tempPosition(2*i-1,:) = relativeMotionHipEE.(EEselection).position(i,:);
end
for i = 1:length(relativeMotionHipEE.(EEselection).velocity)
    tempVelocity(2*i-1,:) = relativeMotionHipEE.(EEselection).velocity(i,:);
end
for i = 1:length(EE.(EEselection).force)
    tempForce(2*i-1,:) = EE.(EEselection).force(i,:);
end
for i = 1:length(C_IBody(1,1,:))
    tempBodyRotation(:,:,2*i-1) = C_IBody(:,:,i);
end

interpolatedPositionData     = fillmissing(tempPosition, 'spline');
interpolatedVelocityData     = fillmissing(tempVelocity, 'linear');
interpolatedForceData        = fillmissing(tempForce, 'linear');
interpolatedBodyRotationData = fillmissing(tempBodyRotation, 'linear', 3);