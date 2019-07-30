% Input spring constant and undeformed angle. A spring is then modeled at
% the joint which deforms by the same angle as the joint. This creates a
% passive torque which together with the actuator's active torque sums to
% the required joint torque.

function [activeTorque, passiveTorque] = getActiveAndPassiveTorque(kSpringJoint, q0SpringJoint, Leg, EEselection, linkCount)
    springDisplacement = Leg.(EEselection).q(:,1:end-1); % q_spring = joint displacement

    % passive torque = kSpring * (q_spring-q_spring0)
    passiveTorque = kSpringJoint.(EEselection)(1:linkCount+1).*(springDisplacement - q0SpringJoint.(EEselection));

    % T_active = T_joint - T_passive
    activeTorque = Leg.(EEselection).jointTorque - passiveTorque;
end