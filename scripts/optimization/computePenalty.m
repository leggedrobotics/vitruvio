function penalty = computePenalty(actuatorProperties, imposeJointLimits, heuristic, legDesignParameters, actuateJointDirectly, linkCount, optimizationProperties, robotProperties, selectFrontHind, taskSelection, dt, configSelection, EEselection, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction, springInParallelWithJoints, kSpringJoint, q0SpringJoint)
    jointNames = ['HAA'; 'HFE'; 'KFE'; 'AFE'; 'DFE'];

    % Design parameters always start with [links(3-5), transmission ratio (3-5), springs parallel to joints (3-5), AFE/DFE spring]    
    % Example for leg like ANYmal: 
    % [hip, thigh, shank, rHAA, rHFE, rKFE]
    jointCount = linkCount+1;
    linkLengths = legDesignParameters(1:jointCount);
    transmissionGearRatio = legDesignParameters(jointCount+1:2*jointCount);
    
    if springInParallelWithJoints
        kSpringJoint.(EEselection) = legDesignParameters(2*jointCount+1:3*jointCount);
    end

    if linkCount > 2 && heuristic.torqueAngle.apply
        kTorsionalSpring = legDesignParameters(end-1);
        thetaLiftoff_des = legDesignParameters(end);
    else 
        kTorsionalSpring = heuristic.torqueAngle.kTorsionalSpring;
        thetaLiftoff_des = heuristic.torqueAngle.thetaLiftoff_des;
    end

    tempLeg.base = Leg.base;
    tempLeg.(EEselection).force = Leg.(EEselection).force;
    tempLeg.basicProperties = Leg.basicProperties;
    tempLeg.actuatorProperties = Leg.actuatorProperties;
    
    % Update robot properties with newly computed leg design parameters, unit in meters
    robotProperties.hip(selectFrontHind).length = linkLengths(1);
    robotProperties.thigh(selectFrontHind).length = linkLengths(2);
    robotProperties.shank(selectFrontHind).length = linkLengths(3);
    if (linkCount == 3) || (linkCount == 4)
        robotProperties.foot(selectFrontHind).length = linkLengths(4);
    end
    if linkCount == 4
        robotProperties.phalanges(selectFrontHind).length = linkLengths(5);
    end

    % Update link mass with assumption of constant density cylinder
    robotProperties.hip(selectFrontHind).mass = robotProperties.legDensity.hip(selectFrontHind) * pi*(robotProperties.hip(selectFrontHind).radius)^2   * abs(linkLengths(1));
    robotProperties.thigh(selectFrontHind).mass = robotProperties.legDensity.thigh(selectFrontHind) * pi*(robotProperties.thigh(selectFrontHind).radius)^2 * abs(linkLengths(2));
    robotProperties.shank(selectFrontHind).mass = robotProperties.legDensity.shank(selectFrontHind) * pi*(robotProperties.shank(selectFrontHind).radius)^2 * abs(linkLengths(3));
    if (linkCount == 3) || (linkCount == 4)
        robotProperties.foot(selectFrontHind).mass = robotProperties.legDensity.foot(selectFrontHind) * pi*(robotProperties.foot(selectFrontHind).radius)^2 * abs(linkLengths(4));
    end
    if linkCount == 4
        robotProperties.phalanges(selectFrontHind).mass = robotProperties.legDensity.phalanges(selectFrontHind) * pi*(robotProperties.phalanges(selectFrontHind).radius)^2 * abs(linkLengths(5));
    end

    tempLeg.robotProperties = robotProperties;
    %% qAFE, qDFE torque based heuristic computation
    if (heuristic.torqueAngle.apply == true) && (linkCount > 2)
        % Save the updated spring parameters back into the heuristic struct to
        % be used in the next iteration of the simulation
        heuristic.torqueAngle.kTorsionalSpring = kTorsionalSpring;
        heuristic.torqueAngle.thetaLiftoff_des = thetaLiftoff_des;
        [qLiftoff.(EEselection)] = computeqLiftoffFinalJoint(heuristic, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, robotProperties, EEselection, configSelection, hipParalleltoBody);
        EE_force = Leg.(EEselection).force(1,1:3);
        rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(1,2); % rotation of body about inertial y
        qPrevious = qLiftoff.(EEselection);
    else
        qLiftoff.(EEselection) = 0; % if the heuristic does not apply
    end

    %% Check maximum distance and skip if leg cannot reach trajectory
    % If max distance is beyond reach of the leg then penalize the design and
    % skip to the end. This simplifies things as does not consider body
    % rotation but should catch most cases and save computation time.

    if hipParalleltoBody && selectFrontHind == 1 % Hip parallel, front leg
        HFEPosition = [linkLengths(1), 0, 0];
    elseif hipParalleltoBody && selectFrontHind == 2 % Hip parallel, hind leg
        HFEPosition = [-linkLengths(1), 0, 0];
    else
        HFEPosition = [0, 0, linkLengths(1)]; % Hip vertically offset from HAA
    end

    distanceHFEToTrajectory = vecnorm(HFEPosition - meanCyclicMotionHipEE.(EEselection).position, 2, 2);
    maxDistanceHFEToTrajectory = max(distanceHFEToTrajectory);
    maxLegExtension = sum(linkLengths(2:end));
    
    if maxDistanceHFEToTrajectory > maxLegExtension
        penalty = 100;
        %disp('Iteration skipped due to tracking error')
    else 
        
        %% Inverse kinematics
        [tempLeg.(EEselection).q, tempLeg.(EEselection).r.HAA, tempLeg.(EEselection).r.HFE, tempLeg.(EEselection).r.KFE, tempLeg.(EEselection).r.AFE, tempLeg.(EEselection).r.DFE, tempLeg.(EEselection).r.EE] = inverseKinematics(tempLeg, heuristic, qLiftoff, meanCyclicMotionHipEE, EEselection);

        %% Build robot model with joint angles from inverse kinematics tempLeg
        gravitySwing  = [0 0 -9.81]; 
        tempLeg.(EEselection).rigidBodyModelSwing  = buildRobotRigidBodyModel(gravitySwing, actuatorProperties, actuateJointDirectly, linkCount, robotProperties, tempLeg, EEselection);
        
        %% Get joint velocities and accelerations with finite differences
        [tempLeg.(EEselection).qdot, tempLeg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(EEselection, tempLeg, dt);
        tempLeg.(EEselection).q = tempLeg.(EEselection).q(1:end-2,:); % remove the two supplementary points for position after solving for joint speed and acceleration 
        
        %% Get joint torques
        externalForce = Leg.(EEselection).force(:,1:3);
        tempLeg.(EEselection).jointTorqueStance = getStanceJointTorques(externalForce, tempLeg, EEselection, meanCyclicMotionHipEE);
        tempLeg.(EEselection).jointTorqueSwing = getSwingJointTorques(EEselection, tempLeg, meanCyclicMotionHipEE, linkCount);
        tempLeg.(EEselection).jointTorque = tempLeg.(EEselection).jointTorqueStance + tempLeg.(EEselection).jointTorqueSwing;

        %% Mechanical power at joint
        jointPowerInitial = Leg.(EEselection).jointTorque .* Leg.(EEselection).qdot(:,1:end-1);
        tempLeg.(EEselection).jointPower = tempLeg.(EEselection).jointTorque .* tempLeg.(EEselection).qdot(:,1:end-1);
        jointPower = tempLeg.(EEselection).jointPower;
        
        % Joint power with positive and negative terms (used to compute
        % antagonistic power)
        jointPowerFullInitial = jointPowerInitial;
        jointPowerFull        = jointPower;
        
        % Set negative terms to zero
        jointPowerInitial(jointPowerInitial<0) = 0;
        jointPower(jointPower<0) = 0;

        if springInParallelWithJoints
            q0SpringJoint.(EEselection) = mean(tempLeg.(EEselection).q(:,1:end-1)); % Set undeformed spring position to mean position. This can be updated in optimizer.
            [tempLeg.(EEselection).activeTorque, tempLeg.(EEselection).passiveTorque] = getActiveAndPassiveTorque(kSpringJoint, q0SpringJoint, Leg, EEselection, linkCount);
            tempLeg.(EEselection).activePower  = tempLeg.(EEselection).activeTorque  .* tempLeg.(EEselection).qdot(:,1:end-1);
            tempLeg.(EEselection).passivePower = tempLeg.(EEselection).passiveTorque .* tempLeg.(EEselection).qdot(:,1:end-1);
        else
            tempLeg.(EEselection).activeTorque = tempLeg.(EEselection).jointTorque;
            tempLeg.(EEselection).passiveTorque = 0;
            q0SpringJoint.(EEselection) = 0;
        end  
        
        %% Get actuator torque and speed as result of gearing between actuator and joint
        % This is the required output torque and speed from the actuator to produce
        % the joint torque and speed.
        for j = 1:linkCount+1
            % gear ratio = actuator speed / joint speed = joint torque/
            % actuator torque
            tempLeg.(EEselection).actuatorq(:,j)      = transmissionGearRatio(j) * tempLeg.(EEselection).q(:,j);
            tempLeg.(EEselection).actuatorqdot(:,j)   = transmissionGearRatio(j) * tempLeg.(EEselection).qdot(:,j);
            tempLeg.(EEselection).actuatorTorque(:,j) = (1/transmissionGearRatio(j)) * tempLeg.(EEselection).activeTorque(:,j);
        end

        %% Get electrical power and efficiency at each operating point
        [tempLeg.(EEselection).electricalPower, tempLeg.(EEselection).operatingPointEfficiency] = computeElectricalPowerInput(tempLeg, EEselection, actuatorProperties, linkCount, actuatorEfficiency, actuatorSelection);

        %% Energy consumption 
        % Here we assume no recuperation of energy possible. This means the
        % negative power terms are set to zero. Integral computed using trapezoids.
        [tempLeg.(EEselection).mechEnergy, tempLeg.metaParameters.mechEnergyPerCycle.(EEselection), tempLeg.(EEselection).elecEnergy, tempLeg.metaParameters.elecEnergyPerCycle.(EEselection)]  = computeEnergyConsumption(jointPower, tempLeg.(EEselection).electricalPower, dt);

        %% Load in penalty weights
        W_totalSwingTorque    = optimizationProperties.penaltyWeight.totalSwingTorque;
        W_totalStanceTorque   = optimizationProperties.penaltyWeight.totalStanceTorque;
        W_totalTorque         = optimizationProperties.penaltyWeight.totalTorque;
        W_totalTorqueHFE      = optimizationProperties.penaltyWeight.totalTorqueHFE;
        W_totalTorqueKFE      = optimizationProperties.penaltyWeight.totalTorqueKFE;
        W_swingTorqueHFE      = optimizationProperties.penaltyWeight.swingTorqueHFE;
        W_totalActiveTorque   = optimizationProperties.penaltyWeight.totalActiveTorque;
        W_stanceActiveTorque  = optimizationProperties.penaltyWeight.stanceActiveTorque;
        W_totalqdot           = optimizationProperties.penaltyWeight.totalqdot;
        W_totalPower          = optimizationProperties.penaltyWeight.totalPower;
        W_totalMechEnergy     = optimizationProperties.penaltyWeight.totalMechEnergy;
        W_totalElecEnergy     = optimizationProperties.penaltyWeight.totalElecEnergy;
        W_averageEfficiency   = optimizationProperties.penaltyWeight.averageEfficiency;
        W_maxTorque           = optimizationProperties.penaltyWeight.maxTorque;
        W_maxqdot             = optimizationProperties.penaltyWeight.maxqdot;
        W_maxPower            = optimizationProperties.penaltyWeight.maxPower;
        W_mechCoT             = optimizationProperties.penaltyWeight.mechCoT;
        W_antagonisticPower   = optimizationProperties.penaltyWeight.antagonisticPower;
        allowableExtension    = optimizationProperties.allowableExtension; % as ratio of total possible extension

        % initialize torque, qdot and power limits.
        maxTorqueLimit = [0 0 0 0 0];    
        maxqdotLimit   = [0 0 0 0 0];    
        maxPowerLimit  = [0 0 0 0 0]; 

        for i = 1:linkCount+1
            jointSelection = jointNames(i,:);
            if imposeJointLimits.maxTorque
                maxTorqueLimit(1,i) = optimizationProperties.bounds.maxTorqueLimit.(jointSelection);
            end

            if imposeJointLimits.maxqdot
                maxqdotLimit(1,i) = optimizationProperties.bounds.maxqdotLimit.(jointSelection);
            end

            if imposeJointLimits.maxPower
                maxPowerLimit(1,i) = optimizationProperties.bounds.maxPowerLimit.(jointSelection);
            end
        end

        %% Initialize penalty terms
        totalTorque        = 0;  totalTorqueInitial        = 1;
        totalTorqueHFE     = 0;  totalTorqueHFEInitial     = 1;
        totalTorqueKFE     = 0;  totalTorqueKFEInitial     = 1;
        swingTorqueHFE     = 0;  swingTorqueHFEInitial     = 1;
        totalSwingTorque   = 0;  totalSwingTorqueInitial   = 1;
        totalStanceTorque  = 0;  totalStanceTorqueInitial  = 1;
        totalActiveTorque  = 0;  totalActiveTorqueInitial  = 1;
        stanceActiveTorque = 0;  stanceActiveTorqueInitial = 1;
        totalqdot          = 0;  totalqdotInitial          = 1;
        totalPower         = 0;  totalPowerInitial         = 1;
        maxTorque          = 0;  maxTorqueInitial          = 1;
        maxqdot            = 0;  maxqdotInitial            = 1;
        maxPower           = 0;  maxPowerInitial           = 1;
        totalMechEnergy    = 0;  totalMechEnergyInitial    = 1;
        totalElecEnergy    = 0;  totalElecEnergyInitial    = 1;
        averageEfficiency  = 1;  averageEfficiencyInitial  = 1;
        antagonisticPower  = 0;  antagonisticPowerInitial  = 1;
        mechCoT            = 0;  mechCoTInitial            = 1;
        
        maximumExtensionPenalty = 0;

        %% Compute penalty terms      
        touchdownIndex = round(Leg.(EEselection).tTouchdown/dt); % only when we average motion
        if W_totalSwingTorque>0
            if linkCount>2 && heuristic.torqueAngle.apply % Exclude torque at final joint because the torque there is passive torque
                torqueInitial.swing  = Leg.(EEselection).actuatorTorque(1:touchdownIndex, 1:end-1);
                torque.swing  = tempLeg.(EEselection).actuatorTorque(1:touchdownIndex, 1:end-1);
            else
                torqueInitial.swing  = Leg.(EEselection).actuatorTorque(1:touchdownIndex, :);
                torque.swing  = tempLeg.(EEselection).actuatorTorque(1:touchdownIndex, :);
            end
            totalSwingTorqueInitial = sum(sum((torqueInitial.swing).^2));
            totalSwingTorque = sum(sum((torque.swing).^2)); 
        end

        if W_totalStanceTorque>0
            if linkCount>2 && heuristic.torqueAngle.apply % Exclude torque at final joint because the torque there is passive torque    
                torqueInitial.stance = Leg.(EEselection).actuatorTorque(touchdownIndex+1:end, 1:end-1);
                torque.stance = tempLeg.(EEselection).actuatorTorque(touchdownIndex+1:end, 1:end-1);
            else
                torqueInitial.stance = Leg.(EEselection).actuatorTorque(touchdownIndex+1:end, :);
                torque.stance = tempLeg.(EEselection).actuatorTorque(touchdownIndex+1:end, :);
            end
            totalStanceTorqueInitial = sum(sum(abs(torqueInitial.stance)));
            totalStanceTorque = sum(sum(abs(torque.stance)));
        end

        if W_totalTorque>0
            if linkCount>2 && heuristic.torqueAngle.apply % Exclude torque at final joint because the torque there is passive torque
                totalTorqueInitial = sum(sum(abs(Leg.(EEselection).actuatorTorque(:,1:end-1)))); 
                totalTorque = sum(sum(abs(tempLeg.(EEselection).actuatorTorque(:,1:end-1))));
            else
                totalTorqueInitial = sum(sum(abs(Leg.(EEselection).actuatorTorque))); 
                totalTorque = sum(sum(abs(tempLeg.(EEselection).actuatorTorque))); 
            end
        end

        if W_totalActiveTorque>0
            totalActiveTorqueInitial = sum(sum(abs(Leg.(EEselection).activeTorque))); 
            totalActiveTorque        = sum(sum(abs(tempLeg.(EEselection).activeTorque))); 
        end
        
        if W_stanceActiveTorque>0
            stanceActiveTorqueInitial = sum(sum(abs(Leg.(EEselection).activeTorque(touchdownIndex+1:end, :)))); 
            stanceActiveTorque        = sum(sum(abs(tempLeg.(EEselection).activeTorque(touchdownIndex+1:end, :)))); 
        end
        
        if W_totalTorqueHFE>0
            totalTorqueHFEInitial = sum(abs(Leg.(EEselection).actuatorTorque(:,2))); 
            totalTorqueHFE = sum(abs(tempLeg.(EEselection).actuatorTorque(:,2))); 
        end

        if W_totalTorqueKFE>0
            totalTorqueKFEInitial = sum(abs(Leg.(EEselection).actuatorTorque(:,3))); 
            totalTorqueKFE = sum(abs(tempLeg.(EEselection).actuatorTorque(:,3))); 
        end

        if W_swingTorqueHFE>0
            swingTorqueHFEInitial = sum(abs(torqueInitial.swing(:,2)));
            swingTorqueHFE = sum(abs(torque.swing(:,2)));
        end

        if W_totalqdot>0
            totalqdotInitial     = sum(sum(abs(Leg.(EEselection).actuatorqdot)));
            totalqdot      = sum(sum(abs(tempLeg.(EEselection).actuatorqdot)));
        end

        if W_totalPower>0
            totalPowerInitial    = sum(sum(jointPowerInitial));
            totalPower     = sum(sum(jointPower));
        end

        if W_mechCoT>0
            mechCoTInitial = Leg.metaParameters.CoT.(EEselection);
            mechCoT = getCostOfTransport(Leg.CoM.meanVelocity, jointPower, robotProperties);
        end
        
        if W_totalMechEnergy>0
            if linkCount>2 && heuristic.torqueAngle.apply   
                totalMechEnergyInitial = sum(Leg.(EEselection).mechEnergy(end,1:end-1)); % sum of mech energy consumed over all active joints during the motion
                totalMechEnergy    = sum(tempLeg.(EEselection).mechEnergy(end,1:end-1));
            else
                totalMechEnergyInitial = sum(Leg.(EEselection).mechEnergy(end,:));
                totalMechEnergy    = sum(tempLeg.(EEselection).mechEnergy(end,:));
            end
        end

        if W_totalElecEnergy>0
            if linkCount>2 && heuristic.torqueAngle.apply     
                totalElecEnergyInitial    = sum(Leg.(EEselection).elecEnergy(end,1:end-1)); % sum of elec energy consumed over all active joints during the motion
                totalElecEnergy    = sum(tempLeg.(EEselection).elecEnergy(end,1:end-1));
            else
                totalElecEnergyInitial    = sum(Leg.(EEselection).elecEnergy(end,:)); % sum of elec energy consumed over all joints during the motion
                totalElecEnergy    = sum(tempLeg.(EEselection).elecEnergy(end,:));        
            end

        end

        if W_averageEfficiency>0    
            averageEfficiencyInitial = mean(mean(Leg.(EEselection).operatingPointEfficiency));
            averageEfficiency = mean(mean(tempLeg.(EEselection).operatingPointEfficiency));
        end

        if W_maxTorque>0
            if linkCount>2 && heuristic.torqueAngle.apply % Exclude spring torque in computation as this is only a passive torque        
                maxTorqueInitial = max(max(abs(Leg.(EEselection).actuatorTorque(:,1:end-1))));
                maxTorque = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,1:end-1))));
            else
                maxTorqueInitial = max(max(abs(Leg.(EEselection).actuatorTorque)));
                maxTorque = max(max(abs(tempLeg.(EEselection).actuatorTorque)));        
            end
        end

        if W_maxqdot>0
            if linkCount>2 && heuristic.torqueAngle.apply             
                maxqdotInitial = max(max(abs(Leg.(EEselection).actuatorqdot(:,1:end-1))));
                maxqdot = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,1:end-1))));
            else
                maxqdotInitial = max(max(abs(Leg.(EEselection).actuatorqdot)));
                maxqdot = max(max(abs(tempLeg.(EEselection).actuatorqdot)));
            end
        end

        if W_maxPower>0
            if linkCount>2 && heuristic.torqueAngle.apply                 
                maxPowerInitial = max(max(jointPowerInitial(:,1:end-1)));
                maxPower = max(max(jointPower(:,1:end-1)));
            else
                maxPowerInitial = max(max(jointPowerInitial));
                maxPower = max(max(jointPower));        
            end
        end

        if W_antagonisticPower>0
            antagonisticPowerInitial      = 0.5*(sum(sum(abs(jointPowerFullInitial))) - abs(sum(sum(jointPowerFullInitial))));
            antagonisticPower             = 0.5*(sum(sum(abs(jointPowerFull))) - abs(sum(sum(jointPowerFull))));
        end
        
        %% For KFE configuration of prototype (KFE fixed to thigh)
        % HFE compensates for KFE torque
        if isequal(Leg.basicProperties.classSelection, 'vitruvianBiped')
            tempLeg.(EEselection).actuatorTorque(:,2) =  tempLeg.(EEselection).actuatorTorque(:,2) + tempLeg.(EEselection).actuatorTorque(:,3);
        end
        
        %% Soft constraints due to actuator limits
        % get the maximum actuator torque speed and power for each joint
        % after applying transmission ratios
        maxTorqueHAA = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,1))));
        maxTorqueHFE = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,2))));
        maxTorqueKFE = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,3))));
        maxTorqueAFE = 0;
        maxTorqueDFE = 0;

        maxqdotHAA   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,1))));
        maxqdotHFE   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,2))));
        maxqdotKFE   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,3))));
        maxqdotAFE   = 0;
        maxqdotDFE   = 0;
        
        % actuator mechanical power = joint power
        maxPowerHAA  = max(max(jointPower(:,1)));
        maxPowerHFE  = max(max(jointPower(:,2)));
        maxPowerKFE  = max(max(jointPower(:,3)));
        maxPowerAFE  = 0;
        maxPowerDFE  = 0;

        % overwrite the AFE and DFE values for 3 and 4 link leg
        if linkCount == 3
            maxTorqueAFE = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,4))));
            maxqdotAFE   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,4))));
            maxPowerAFE  = max(max(jointPower(:,4)));
        end
        if linkCount == 4
            maxTorqueAFE = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,4))));
            maxTorqueDFE = max(max(abs(tempLeg.(EEselection).actuatorTorque(:,5))));
            maxqdotAFE   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,4))));
            maxqdotDFE   = max(max(abs(tempLeg.(EEselection).actuatorqdot(:,5))));
            maxPowerAFE  = max(max(jointPower(:,4)));
            maxPowerDFE  = max(max(jointPower(:,5)));
        end

        maxActuatorTorque = [maxTorqueHAA, maxTorqueHFE, maxTorqueKFE, maxTorqueAFE, maxTorqueDFE];
        maxActuatorqdot   = [maxqdotHAA,   maxqdotHFE,   maxqdotKFE,   maxqdotAFE,   maxqdotDFE];
        maxJointPower     = [maxPowerHAA,  maxPowerHFE,  maxPowerKFE,  maxPowerAFE,  maxPowerDFE];

        %% Compute max joint torque, speed and power limit violation penalties
        % initialize penalties for exceeding actuator limits
        torqueLimitPenalty = 0; speedLimitPenalty = 0; powerLimitPenalty = 0;

        if imposeJointLimits.maxTorque && any(maxActuatorTorque > imposeJointLimits.limitingValue * maxTorqueLimit)
            torqueViolation = (maxActuatorTorque(1:jointCount) - maxTorqueLimit(1:jointCount))./maxTorqueLimit(1:jointCount); % violation of limit in percent
            torqueViolation(torqueViolation<0)=0;
            torqueLimitPenalty = 1+sum(torqueViolation);
            %disp('Exceeded torque limit')
        end

        if imposeJointLimits.maxqdot && any(maxActuatorqdot > imposeJointLimits.limitingValue * maxqdotLimit)
            speedViolation = (maxActuatorqdot(1:jointCount) - maxqdotLimit(1:jointCount))./maxqdotLimit(1:jointCount);
            speedViolation(speedViolation<0)=0;
            speedLimitPenalty = 1+sum(speedViolation);
            %disp('Exceeded speed limit')
        end

        if imposeJointLimits.maxPower && any(maxJointPower > imposeJointLimits.limitingValue * maxPowerLimit)
            powerViolation = (maxJointPower(1:jointCount) - maxPowerLimit(1:jointCount))./maxPowerLimit(1:jointCount);
            powerViolation(powerViolation<0)=0;
            powerLimitPenalty = 1+sum(powerViolation);            
            %disp('Exceeded power limit')
        end

        %% Tracking error penalty
        % impose tracking error penalty if any point has tracking error above an
        % allowable threshhold
        trackingError = meanCyclicMotionHipEE.(EEselection).position(1:length(tempLeg.(EEselection).r.EE),:)-tempLeg.(EEselection).r.EE;
        trackingError = sqrt(sum(trackingError.^2,2)); % Magnitude of tracking error at each timestep
        if max(trackingError) > 0.001 % If tracking error ever exceeds this tolerance, impose a penalty.
            trackingErrorPenalty = 10;
            %disp('Large tracking error')
        else
            trackingErrorPenalty = 0;
        end

        %% Joint too close to ground
        % Requires us to bring in hipNomZ
        % For flat ground, ground height is the distance CoM z position.
        groundHeight = -Leg.base.position.(EEselection)(:,3);
        allowableHeightAboveGround = 0; % m
        % Get the height of the lowest joint at each time step.
        for i = 1:length(tempLeg.(EEselection).r.HAA(:,3))
            jointHeight = []; % Initialize jointHeight to be empty
            for j = 1:linkCount+1 
                % Fill in joint heights for all of the joints in the leg.
                jointHeight = [jointHeight, tempLeg.(EEselection).r.(jointNames(j,:))(i,3)];         
            end
            % return the lowest point of all joints at time i
            lowestJoint(i,1) =  min(jointHeight);
        end              
        % Apply penalty if the lowest joint is ever too close to the ground                 
        if any(lowestJoint - groundHeight(1:length(lowestJoint)) < allowableHeightAboveGround)
            jointBelowGroundPenalty = 10;
            %disp('Joint below ground')
        else
            jointBelowGroundPenalty = 0;
        end

        %% KFE above HFE penalty - otherwise spider config preferred
        % find max z position of KFE and penalize if above origin
       jointHeightHFE = tempLeg.(EEselection).r.HFE(:,3);
       jointHeightKFE = tempLeg.(EEselection).r.KFE(:,3);

        % Apply penalty if KFE joint is ever above HFE joint
        if any(jointHeightKFE > jointHeightHFE)
            KFEAboveHFEPenalty = 0;
            %disp('KFE joint above HFE joint')
        else
            KFEAboveHFEPenalty = 0;
        end

        %% AFE/DFE in front of end effector (Control spring deform. direction)
        % Applies only if using spring heuristic
        if linkCount>2 && heuristic.torqueAngle.apply                 
            for i = 1:length(tempLeg.(EEselection).r.HAA(:,3))
                if linkCount == 3
                    jointPositionxLastJoint(i,1) = tempLeg.(EEselection).r.AFE(i,1);
                else
                    jointPositionxLastJoint(i,1) = tempLeg.(EEselection).r.DFE(i,1);
                end            
            end   
            % Apply penalty if last joint (AFE or DFE) is ever in front of EE
            if any(jointPositionxLastJoint > tempLeg.(EEselection).r.EE(i,1))
                lastJointInFrontOfEEPenalty = 10;
            else
                lastJointInFrontOfEEPenalty = 0;
            end 
        else
            lastJointInFrontOfEEPenalty = 0;
        end

        %% Overextension penalty
        if optimizationProperties.penaltyWeight.maximumExtension % if true, calculate and penalize for overzealous extension
            offsetHFE2EEdes = tempLeg.(EEselection).r.HFE - meanCyclicMotionHipEE.(EEselection).position(1:end-2,:); % offset from HFE to desired EE position at all time steps
            maxOffsetHFE2EEdes = max(sqrt(sum(offsetHFE2EEdes.^2,2))); % max euclidian distance from HFE to desired EE position
            if maxOffsetHFE2EEdes > allowableExtension*sum(linkLengths(2:end))
                maximumExtensionPenalty = 10;
                %disp('Maximum allowable extension exceeded')
            else 
                maximumExtensionPenalty = 0;
            end
        end

        %% Compute total penalty as sum of optimization goal related penalty terms and soft constraints
        penalty = W_totalTorque         * (totalTorque/totalTorqueInitial) + ...
                  W_totalTorqueHFE      * (totalTorqueHFE/totalTorqueHFEInitial) + ...
                  W_totalTorqueKFE      * (totalTorqueKFE/totalTorqueKFEInitial) + ...          
                  W_swingTorqueHFE      * (swingTorqueHFE/swingTorqueHFEInitial) + ...
                  W_totalSwingTorque    * (totalSwingTorque/totalSwingTorqueInitial) + ...
                  W_totalStanceTorque   * (totalStanceTorque/totalStanceTorqueInitial) + ...
                  W_totalActiveTorque   * (totalActiveTorque/totalActiveTorqueInitial) + ...
                  W_stanceActiveTorque  * (stanceActiveTorque/stanceActiveTorqueInitial) + ...
                  W_totalqdot           * (totalqdot/totalqdotInitial)     + ...
                  W_totalPower          * (totalPower/totalPowerInitial)   + ...
                  W_maxTorque           * (maxTorque/maxTorqueInitial)     + ...
                  W_maxqdot             * (maxqdot/maxqdotInitial)         + ...
                  W_maxPower            * (maxPower/maxPowerInitial)     + ...
                  W_antagonisticPower   * (antagonisticPower/antagonisticPowerInitial)     + ... 
                  W_mechCoT             * (mechCoT/mechCoTInitial)          + ...
                  W_totalMechEnergy     * (totalMechEnergy/totalMechEnergyInitial)     + ... 
                  W_totalElecEnergy     * (totalElecEnergy/totalElecEnergyInitial)     + ...
                  W_averageEfficiency   / (averageEfficiency/averageEfficiencyInitial)     + ... % minimize 1/efficiency
                  trackingErrorPenalty        + ...
                  jointBelowGroundPenalty     + ...
                  KFEAboveHFEPenalty          + ...
                  lastJointInFrontOfEEPenalty + ... % only applies for spring at final joint to control the spring deformation direction
                  optimizationProperties.penaltyWeight.maximumExtension * maximumExtensionPenalty + ...
                  10*torqueLimitPenalty + ...
                  10*speedLimitPenalty  + ...
                  10*powerLimitPenalty;
    end
end