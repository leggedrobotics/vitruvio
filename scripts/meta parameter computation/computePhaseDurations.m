function [swingDuration, stanceDuration, swingDurationRatio] = computePhaseDurations(tLiftoff, tTouchdown, EEselection)
swing = tTouchdown.(EEselection) - tLiftoff.(EEselection); % time at touchdown less the time at corresponding liftoff
swingDuration = mean(swing); % seconds

for i = 1:length(tTouchdown.(EEselection)) - 1
    stance(i) = tLiftoff.(EEselection)(i+1) - tTouchdown.(EEselection)(i); % time at liftoff less the time at previous touchdown
end
stanceDuration = mean(stance); % seconds
swingDurationRatio = swingDuration/(swingDuration+stanceDuration); % as ratio of total cycle duration