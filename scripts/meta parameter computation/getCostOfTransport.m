function CoT = getCostOfTransport(Leg, power, quadruped)
% CoT = power / m*g*v
    m = quadruped.mass.total;
    g = 9.81;
    % mean velocity in x direction
    v = mean(Leg.CoM.velocity(:,1));

    % set negative power to zero, no regeneration
    power(power<0) = 0;

    % sum over all actuators of the mean joint power at that actuator over the entire motion 
    meanJointPowerForMotion = sum(mean(power));
    CoT = meanJointPowerForMotion/(m*g*v);
end