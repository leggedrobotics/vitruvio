function [neighbouringPointDeviation, neighbouringPointDeviationMax] = getDeviationFromNeighbouringPoint(relativeMotionHipEE, EEselection)
        % compute offset between neighbouring points as well as between the
        % end point and first point of the cycle
        neighbouringPointOffset = relativeMotionHipEE.(EEselection).position(1:end-1,:) - relativeMotionHipEE.(EEselection).position(2:end,:);

    for i = 1:length(neighbouringPointOffset)-1
        neighbouringPointDeviation(i,:) = norm(neighbouringPointOffset(i));
    end
    [neighbouringPointDeviationMax, index] = max(neighbouringPointDeviation);
end


        