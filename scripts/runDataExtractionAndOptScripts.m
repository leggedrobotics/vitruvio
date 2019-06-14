function Leg = runDataExtractionAndOptScripts(applyHeuristic, kTorsionalSpring, thetaLiftoff_des, actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody)

EEnames = ['LF'; 'RF'; 'LH'; 'RH'];
linkNames = {'hip','thigh' 'shank' 'foot' 'phalanges'};
l_hipAttachmentOffset.fore = -0.14; % this will become an optimization parameter but for now is hard coded. Need one for hind and one for front legs.
l_hipAttachmentOffset.hind = 0.14;
fprintf('Loading data for task %s.\n', taskSelection);

% Get suggested removal ratio for cropping motion data to useful steady state motion
[removalRatioStart, removalRatioEnd] = getSuggestedRemovalRatios(taskSelection);

% Load motion and force data from .mat file
load(taskSelection);

%% Load corresponding robot parameters
fprintf('Loading quadruped properties for %s.\n', classSelection);
quadruped = getQuadrupedProperties(classSelection, linkCount);

for i = 1:4
    EEselection = EEnames(i,:);
     if (EEselection == 'LF') | (EEselection == 'RF')
         selectFrontHind = 1;
     else
         selectFrontHind = 2;
     end
     for j = 1:linkCount+1
        Leg.(EEselection).linkLengths(1,j) = quadruped.(linkNames{j})(selectFrontHind).length;
    end
end

%% Get the relative motion of the end effectors to the hips
fprintf('Computing motion of end effectors relative to hip attachment points. \n');
[relativeMotionHipEE, IF_hip, C_IBody] = getRelativeMotionEEHips(quat, quadruped, base, EE, dt);

%% Get the liftoff and touchdown timings for each end effector
dt = t(2) - t(1);
fprintf('Computing end effector liftoff and touchdown timings. \n');
[tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(t, EE);

%% Get the mean cyclic position and forces for each end effector
fprintf('Computing average relative motion of end effectors over one step. \n');
[meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd, meanTouchdownIndex] = getHipEECyclicData(quadruped, tLiftoff, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody, EEnames);
for i = 1:4
    EEselection = EEnames(i,:);
    Leg.(EEselection).force = meanCyclicMotionHipEE.(EEselection).force;
end

% this is a temporary solution to the issue of phase shift between end
% effector and body, particularly for speedy gallop. For stair climbing robots, the body rotates but the
% rotation is neglected for flat ground.
if abs(mean(meanCyclicMotionHipEE.body.eulerAngles(:,2))) < 0.2
    meanCyclicMotionHipEE.body.eulerAngles = zeros(length(meanCyclicMotionHipEE.body.eulerAngles),3);
end
%% Get reachable positions for link lengths and joint limits
fprintf('Computing range of motion dependent on link lengths and joint limits. \n');
reachablePositions = getRangeofMotion(quadruped);

%% Plot trajectory data
if viewTrajectoryPlots
    fprintf('Plotting data. \n');
    plotMotionData;
end

%% qAFE, qDFE force based heuristic computation
% computes the angle of the final joint at all time steps by deformation
% approximated using the EE force in z at the timestep
if (linkCount > 2)
    if applyHeuristic.forceAngle == true
        fprintf('Computing AFE/DFE angle using force based heuristic.\n');
        for i = 1:4
            EEselection = EEnames(i,:);
            if linkCount == 3
                Leg.(EEselection).q(:,4) = computeqFinalJoint(Leg, EEselection, configSelection);
            else
                Leg.(EEselection).q(:,5) = computeqFinalJoint(Leg, EEselection, configSelection);
            end
        end
    end
end

%% qAFE, qDFE torque based heuristic computation
% compute the liftoff angle between the final link and the horizontal to
% satisfy the input requirement of thetaLiftoff_des. If this heuristic is
% used, the IK is solved using joint torque from previous timestep to
% define the joint deformation at the current timestep.
if (applyHeuristic.torqueAngle == true) && (linkCount > 2)
    fprintf('Computing joint angles to for desired EE liftoff angle. \n');
    for i = 1:4
        EEselection = EEnames(i,:);
        if (EEselection == 'LF') | (EEselection == 'RF')
            hipAttachmentOffset = l_hipAttachmentOffset.fore;
        elseif (EEselection == 'LH') | (EEselection == 'RH')
            hipAttachmentOffset = l_hipAttachmentOffset.hind;
        end
        [qLiftoff.(EEselection)] = computeqLiftoffFinalJoint(thetaLiftoff_des, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody, Leg);
        EE_force = Leg.(EEselection).force(1,1:3);
        rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles(i,2); % rotation of body about inertial y
        qPrevious = qLiftoff.(EEselection);
        [springTorque.(EEselection), springDeformation.(EEselection)] = computeFinalJointDeformation(kTorsionalSpring, qPrevious, EE_force, hipAttachmentOffset, linkCount, rotBodyY, quadruped, EEselection, hipParalleltoBody);      
    end
    else
        qLiftoff.(EEselection) = 0;
end


%% inverse kinematics
fprintf('Computing joint angles using inverse kinematics.\n');
for i = 1:4
    EEselection = EEnames(i,:);
    if (EEselection == 'LF') | (EEselection == 'RF')
        hipAttachmentOffset = l_hipAttachmentOffset.fore;
    elseif (EEselection == 'LH') | (EEselection == 'RH')
        hipAttachmentOffset = l_hipAttachmentOffset.hind;
    end
    [Leg.(EEselection).q, Leg.(EEselection).r.HAA, Leg.(EEselection).r.HFE, Leg.(EEselection).r.KFE, Leg.(EEselection).r.AFE, Leg.(EEselection).r.DFE, Leg.(EEselection).r.EE]  = inverseKinematics(applyHeuristic, kTorsionalSpring, qLiftoff, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody, Leg);
     Leg.(EEselection).r.EEdes = meanCyclicMotionHipEE.(EEselection).position; % save desired EE position in the same struct for easy comparison
end

%% Build robot rigid body model
fprintf('Creating and visualizing robot rigid body model. \n');
for i = 1:4
    EEselection = EEnames(i,:);
    if (EEselection == 'LF') | (EEselection == 'RF')
        hipAttachmentOffset = l_hipAttachmentOffset.fore;
    elseif (EEselection == 'LH') | (EEselection == 'RH')
        hipAttachmentOffset = l_hipAttachmentOffset.hind;
    end
    Leg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(actuateJointsDirectly, hipAttachmentOffset, linkCount, quadruped, Leg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody);
end

%% Get joint velocities with inverse(Jacobian)* EE.velocity
% the joint accelerations are then computed using finite difference
fprintf('Computing joint velocities and accelerations. \n');
for i = 1:4
    EEselection = EEnames(i,:);
    [Leg.(EEselection).qdot, Leg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(linkCount, EEselection, meanCyclicMotionHipEE, Leg, quadruped, dt);
    Leg.(EEselection).q = Leg.(EEselection).q(1:end-2,:); % no longer need the two additional points for position after solving for joint speed and acceleration
end

%% Get joint torques using inverse dynamics
fprintf('Computing joint torques using inverse dynamics. \n');
for i = 1:4
    EEselection = EEnames(i,:);
    Leg.(EEselection).jointTorque = inverseDynamics(EEselection, Leg, meanCyclicMotionHipEE, linkCount);
end

%% Get joint power
fprintf('Computing joint power. \n');
for i = 1:4
    EEselection = EEnames(i,:);
    Leg.(EEselection).jointPower = Leg.(EEselection).jointTorque .* Leg.(EEselection).qdot(:,1:end-1);
end

%% Save link lengths and mass
for i = 1:4
    EEselection = EEnames(i,:);
    [Leg.(EEselection).linkMass, Leg.(EEselection).EEMass, Leg.(EEselection).totalLinkMass] = getLinkMass(Leg, EEselection, linkCount);
    if (EEselection == 'LF') | (EEselection == 'RF')
        Leg.(EEselection).hipOffset = quadruped.hipOffset(1);
    else
         Leg.(EEselection).hipOffset = quadruped.hipOffset(2);
    end
end

%% Get meta parameters
Leg.CoM.velocity = base.velocity(floor(removalRatioStart*length(base.velocity))+1:floor((1-removalRatioEnd)*length(base.velocity)),:);
Leg.metaParameters.CoT.total = 0;
Leg.metaParameters.energyPerCycleTotal = 0;
for i = 1:4
    EEselection = EEnames(i,:);
    power = Leg.(EEselection).jointPower;
    Leg.metaParameters.CoT.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
%     add contribution of each leg to get the total CoT
    Leg.metaParameters.CoT.total = Leg.metaParameters.CoT.total + Leg.metaParameters.CoT.(EEselection); 
    [Leg.metaParameters.deltaqMax.(EEselection), Leg.metaParameters.qdotMax.(EEselection), Leg.metaParameters.jointTorqueMax.(EEselection), Leg.metaParameters.jointPowerMax.(EEselection), Leg.(EEselection).energy, Leg.metaParameters.energyPerCycle.(EEselection)]  = getMaximumJointStates(Leg, power, EEselection, dt);
    [Leg.metaParameters.swingDuration.(EEselection), Leg.metaParameters.stanceDuration.(EEselection), Leg.metaParameters.swingDurationRatio.(EEselection)] = computePhaseDurations(tLiftoff, tTouchdown, EEselection);
    Leg.metaParameters.energyPerCycleTotal = Leg.metaParameters.energyPerCycleTotal + sum(Leg.metaParameters.energyPerCycle.(EEselection));
end

%% Optimize selected legs and compute their cost of transport
if runOptimization
    Leg.metaParameters.CoTOpt.total = 0;
    if optimizeLF
        EEselection = 'LF';
        hipAttachmentOffset = l_hipAttachmentOffset.fore;
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
         % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).hipOffsetOpt, Leg.(EEselection).penaltyMinOpt, Leg.metaParameters.elapsedTime.(EEselection), Leg.metaParameters.(EEselection).elapsedTimePerFuncEval, Leg.metaParameters.(EEselection).output, Leg.(EEselection).linkMassOpt, Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(actuateJointsDirectly,hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg, meanTouchdownIndex);
         Leg.(EEselection).qOpt = Leg.(EEselection).qOpt(1:end-2,:); % after solving finite difference we can remove the additional looped around terms from the position vector,
         % compute CoT 
         power = Leg.(EEselection).jointPowerOpt;
         Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
        [Leg.metaParameters.deltaqMaxOpt.(EEselection), Leg.metaParameters.qdotMaxOpt.(EEselection), Leg.metaParameters.jointTorqueMaxOpt.(EEselection), Leg.metaParameters.jointPowerMaxOpt.(EEselection), Leg.(EEselection).energyOpt, Leg.metaParameters.energyPerCycleOpt.(EEselection)]  = getMaximumJointStates(Leg, power, EEselection, dt);
    end 
    
    if optimizeLH
        EEselection = 'LH';
        hipAttachmentOffset = l_hipAttachmentOffset.hind;        
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).hipOffsetOpt, Leg.(EEselection).penaltyMinOpt, Leg.metaParameters.elapsedTime.(EEselection), Leg.(EEselection).metaParameters.elapsedTimePerFuncEval, Leg.(EEselection).metaParameters.output, Leg.(EEselection).linkMassOpt,  Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg, meanTouchdownIndex);
         Leg.(EEselection).qOpt = Leg.(EEselection).qOpt(1:end-2,:); % after solving finite difference we can remove the additional looped around terms from the position vectors
        % compute CoT
         power = Leg.(EEselection).jointPowerOpt;
         Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
        [Leg.metaParameters.deltaqMaxOpt.(EEselection), Leg.metaParameters.qdotMaxOpt.(EEselection), Leg.metaParameters.jointTorqueMaxOpt.(EEselection), Leg.metaParameters.jointPowerMaxOpt.(EEselection), Leg.(EEselection).energyOpt, Leg.metaParameters.energyPerCycleOpt.(EEselection)]  = getMaximumJointStates(Leg, power, EEselection, dt);         
    end
    
    if optimizeRF
        EEselection = 'RF';
        hipAttachmentOffset = l_hipAttachmentOffset.fore;      
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).hipOffsetOpt, Leg.(EEselection).penaltyMinOpt, Leg.metaParameters.elapsedTime.(EEselection), Leg.(EEselection).metaParameters.elapsedTimePerFuncEval, Leg.(EEselection).metaParameters.output, Leg.(EEselection).linkMassOpt,  Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg, meanTouchdownIndex);
         Leg.(EEselection).qOpt = Leg.(EEselection).qOpt(1:end-2,:); % after solving finite difference we can remove the additional looped around terms from the position vectors
        % compute CoT
         power = Leg.(EEselection).jointPowerOpt;
         Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
        [Leg.metaParameters.deltaqMaxOpt.(EEselection), Leg.metaParameters.qdotMaxOpt.(EEselection), Leg.metaParameters.jointTorqueMaxOpt.(EEselection), Leg.metaParameters.jointPowerMaxOpt.(EEselection), Leg.(EEselection).energyOpt, Leg.metaParameters.energyPerCycleOpt.(EEselection)]  = getMaximumJointStates(Leg, power, EEselection, dt);
    end
    
    if optimizeRH
        EEselection = 'RH';
        hipAttachmentOffset = l_hipAttachmentOffset.hind;
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).hipOffsetOpt, Leg.(EEselection).penaltyMinOpt, Leg.metaParameters.elapsedTime.(EEselection), Leg.metaParameters.(EEselection).elapsedTimePerFuncEval, Leg.(EEselection).metaParameters.output, Leg.(EEselection).linkMassOpt,  Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg, meanTouchdownIndex);
         Leg.(EEselection).qOpt = Leg.(EEselection).qOpt(1:end-2,:); % after solving finite difference we can remove the additional looped around terms from the position vectors
        % compute CoT
         power = Leg.(EEselection).jointPowerOpt;
         Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
        [Leg.metaParameters.deltaqMaxOpt.(EEselection), Leg.metaParameters.qdotMaxOpt.(EEselection), Leg.metaParameters.jointTorqueMaxOpt.(EEselection), Leg.metaParameters.jointPowerMaxOpt.(EEselection), Leg.(EEselection).energyOpt, Leg.metaParameters.energyPerCycleOpt.(EEselection)]  = getMaximumJointStates(Leg, power, EEselection, dt);
    end 
    
    % total cost of transport is the sum of each leg component
    if optimizeLF && optimizeLH && optimizeRF && optimizeRH
        for i = 1:4
            EEselection = EEnames(i,:);
            Leg.metaParameters.CoTOpt.total = Leg.metaParameters.CoTOpt.total + Leg.metaParameters.CoTOpt.(EEselection);

        end
    end
        
%% plot joint torque and speed for initial and optimized design
    if viewOptimizedLegPlot
        fprintf('Plotting joint data for initial and optimized leg designs \n');
        if optimizeLF
            EEselection = 'LF';
            plotOptimizedJointTorque(Leg, EEselection, dt, meanTouchdownIndex, taskSelection, linkCount)
        end
        if optimizeLH
            EEselection = 'LH';
            plotOptimizedJointTorque(Leg, EEselection, dt, meanTouchdownIndex, taskSelection, linkCount)
        end
        if optimizeRF
            EEselection = 'RF';
            plotOptimizedJointTorque(Leg, EEselection, dt, meanTouchdownIndex, taskSelection, linkCount)
        end
        if optimizeRH
            EEselection = 'RH';
            plotOptimizedJointTorque(Leg, EEselection, dt, meanTouchdownIndex, taskSelection, linkCount)
        end
    end
end
