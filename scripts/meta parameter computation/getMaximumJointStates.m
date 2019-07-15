% get max of joint torque, speed and power
function [deltaqMax, qdotMax, jointTorqueMax, jointPowerMax]  = getMaximumJointStates(Leg, EEselection)    
    qMin = min(Leg.(EEselection).q(:,1:end-1));
    qMax = max(Leg.(EEselection).q(:,1:end-1));
    deltaqMax = qMax - qMin;
    qdotMax = max(abs(Leg.(EEselection).qdot(:,1:end-1)));
    jointTorqueMax = max(abs(Leg.(EEselection).jointTorque));
    jointPowerMax = max(Leg.(EEselection).jointPower);
end