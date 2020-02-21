function CoT = getCostOfTransportPenaltyContribution(v, power, linkMass)
% CoT = power / m*g*v
    m = linkMass;
    g = 9.81;

    % set negative power to zero, no regeneration
    power(power<0) = 0;

    % sum over all actuators of the mean joint power at that actuator over the entire motion 
    meanJointPowerForMotion = sum(mean(power));
    CoT = meanJointPowerForMotion/(m*g*v);
end