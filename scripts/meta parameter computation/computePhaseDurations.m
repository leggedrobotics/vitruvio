function [swingDuration, stanceDuration, swingDurationRatio] = computePhaseDurations(Leg, EEselection)
    swing = Leg.fullTrajectory.tTouchdown.(EEselection) - Leg.fullTrajectory.tLiftoff.(EEselection); % time at touchdown less the time at corresponding liftoff
    swingDuration = mean(swing); % seconds

    for i = 1:length(Leg.fullTrajectory.tTouchdown.(EEselection)) - 1
        stance(i) = Leg.fullTrajectory.tLiftoff.(EEselection)(i+1) - Leg.fullTrajectory.tTouchdown.(EEselection)(i); % time at liftoff less the time at previous touchdown
    end
    
    stanceDuration = mean(stance); % seconds
    
    if swingDuration+stanceDuration == 0 % avoid divide by zero for pushup motion
        stanceDuration = 1;
    end
    swingDurationRatio = swingDuration/(swingDuration+stanceDuration); % as ratio of total cycle duration
end