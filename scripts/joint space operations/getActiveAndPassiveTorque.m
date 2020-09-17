% Input spring constant and undeformed angle. A spring is then modeled at
% the joint which deforms by the same angle as the joint. This creates a
% passive torque which together with the actuator's active torque sums to
% the required joint torque.

function [activeTorque, passiveTorque] = getActiveAndPassiveTorque(kSpringJoint, q0SpringJoint, Leg, EEselection, linkCount)
    qSpring = Leg.(EEselection).q(:,1:end-1); % spring angle = joint angle

    % Hooke's Law
    passiveTorque = -kSpringJoint(1:linkCount+1).*(qSpring - q0SpringJoint);

    % T_active = T_joint - T_passive
    activeTorque = Leg.(EEselection).jointTorque - passiveTorque;
end