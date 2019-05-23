% input EE forces and joint position which defines robot stance at each 
% time step, joint speed and acceleration. Output joint torques required to
% achieve the specified motion given the EE forces

function jointTorque = inverseDynamics(EEselection, Leg, meanCyclicMotionHipEE)
jointTorque = zeros(length(Leg.(EEselection).qdotdot),4);
    for i = 1:length(Leg.(EEselection).qdotdot)
        rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles(i,2);
        q = Leg.(EEselection).q(i,1:3);
        qdot = Leg.(EEselection).qdot(i,1:3);
        qdotdot = Leg.(EEselection).qdotdot(i,:);
        rigidBodyModel = Leg.(EEselection).rigidBodyModel;
        
        wrench = [0 0 0 meanCyclicMotionHipEE.(EEselection).force(i,1:3)]; % torques and forces applied to the body [Tx Ty Tz Fx Fy Fz]
        fext = externalForce(rigidBodyModel,'body4',wrench); % apply force on body4 = end effector
        % we should get same result of torque by setting rotBodyY = 0
        jointTorque(i,:) = inverseDynamics(rigidBodyModel, [rotBodyY q], [0 qdot], [0 qdotdot], fext);
    end
% the first term is due to body rotation but this is not related to an actuated joint so we neglect it    
jointTorque = jointTorque(:,2:end);
end
