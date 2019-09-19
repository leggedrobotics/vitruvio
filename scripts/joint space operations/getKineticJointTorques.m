% Calculate contribution to joint torque due to kinetics by: J'*F = torque
function jointTorque = getKineticJointTorques(externalForce, Leg, EEselection, meanCyclicMotionHipEE)
    for i = 1:length(Leg.(EEselection).q(:,1))
        q        = Leg.(EEselection).q(i,1:end-1);
        rotBodyY = meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(i,2);
        [J_P, C_0EE, r_H_01, r_H_02, r_H_03, r_H_04, r_H_05, r_H_0EE] = jointToPosJac(Leg, rotBodyY, q, EEselection);
        
        torque = J_P'*externalForce(i,:)';
        jointTorque(i,:) = torque(1:end-1); %Last column is torque at EE which is zero
    end
    
    % These are torques from external forces. The actuators need to supply
    % the reaction torques (ie the negative)
    jointTorque = -jointTorque;
end