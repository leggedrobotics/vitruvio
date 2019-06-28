% get max of joint torque, speed and power

function [deltaqMax, qdotMax, jointTorqueMax, jointPowerMax, energy, energyPerCycle]  = getMaximumJointStates(Leg, power, EEselection, dt)
    qMin = min(Leg.(EEselection).q(:,1:end-1));
    qMax = max(Leg.(EEselection).q(:,1:end-1));
    deltaqMax = qMax - qMin;
    qdotMax = max(abs(Leg.(EEselection).qdot(:,1:end-1)));
    jointTorqueMax = max(abs(Leg.(EEselection).jointTorque));
    % set negative power to zero, no regeneration
    for j = 1:length(power)
        for k = 1:length(power(1,:))
            if power(j,k) < 0
                  power(j,k) = 0;
            end
        end
    end
    energy = power*dt; % energy consumed at each timestep of dt
    energyPerCycle = sum(energy);
    jointPowerMax = max(power);
end