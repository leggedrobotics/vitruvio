function [neighbouringPointDistance, neighbouringPointDistanceMax] = getDeviationFromNeighbouringPoint(relativeMotionHipEE, EEselection)
    for i = 1:length(relativeMotionHipEE.(EEselection).position)-1
        neighbouringPointOffset(i,:) = relativeMotionHipEE.(EEselection).position(i,:)-relativeMotionHipEE.(EEselection).position(i+1,:);
        neighbouringPointDistance(i,:) = norm(neighbouringPointOffset(i));
    end
    neighbouringPointDistanceMax = max(neighbouringPointDistance);
end