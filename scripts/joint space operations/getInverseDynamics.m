%% getInverseDynamics

function jointTorque = getInverseDynamics(EEselection, Leg, meanCyclicMotionHipEE)
    for i = 1:length(Leg.(EEselection).qdotdot)
        q = Leg.(EEselection).q(i,1:3);
        qdot = Leg.(EEselection).qdot(i,1:3);
        qdotdot = Leg.(EEselection).qdotdot(i,:);
        rigidBodyModel = Leg.(EEselection).rigidBodyModel;
        
        wrench = [0 0 0 meanCyclicMotionHipEE.(EEselection).force(i,1:3)]; % need to rotate this force to base frame?
        fext = externalForce(rigidBodyModel,'body4',wrench); % apply force on body4 = end effector

        jointTorque(i,:) = inverseDynamics(rigidBodyModel, q, qdot, qdotdot, fext);
    end
end