function Leg = runDataExtractionAndOptScripts(actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody)

%% display selections
fprintf('Quadruped class: %s.\n', classSelection);
fprintf('Task: %s.\n', taskSelection);
fprintf('Actuator: %s.\n', actuatorSelection);
if linkCount == 2
    fprintf('Links: hip, thigh, shank.\n\n')
elseif linkCount == 3
    fprintf('Links: hip, thigh, shank, foot.\n\n')
elseif linkCount == 4
    fprintf('Links: hip, thigh, shank, foot, phalanges.\n\n')
end

%% Load in trajectory
EEnames = ['LF'; 'RF'; 'LH'; 'RH'];
linkNames = {'hip','thigh' 'shank' 'foot' 'phalanges'};

% Get suggested removal ratio for cropping motion data to useful steady state motion
[removalRatioStart, removalRatioEnd] = getSuggestedRemovalRatios(taskSelection);

% Load motion and force data from .mat file
load(taskSelection);
dt = t(2) - t(1);
Leg.time = t;

%% Load corresponding robot parameters
fprintf('Loading quadruped properties for %s.\n', classSelection);
quadruped = getQuadrupedProperties(classSelection, linkCount);
[~, ~, ~, actuatorProperties.mass] = getActuatorProperties(actuatorSelection);

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

%% Get deviation from neighbouring point
for i = 1:4
    EEselection = EEnames(i,:);
    [neighbouringPointDeviation.(EEselection), neighbouringPointDeviationMax.(EEselection)] = getDeviationFromNeighbouringPoint(relativeMotionHipEE, EEselection);
    Leg.metaParameters.neighbouringPointDeviation.(EEselection) = neighbouringPointDeviation.(EEselection);
    Leg.metaParameters.neighbouringPointDeviationMax.(EEselection) = neighbouringPointDeviationMax.(EEselection);
end

%% If points are spread too far apart, interpolate to add more points
allowableDeviation = dataExtraction.allowableDeviation;
absoluteMaxPointDeviation = max([neighbouringPointDeviationMax.LF, neighbouringPointDeviationMax.LH, neighbouringPointDeviationMax.RF, neighbouringPointDeviationMax.RH]);
if absoluteMaxPointDeviation > allowableDeviation
    numberOfTimesPointsAreDoubled = ceil(log(absoluteMaxPointDeviation/allowableDeviation)/log(2)); % because deviation decreases by factor of 2^n
    fprintf('Interpolating points to reduce distance between neighbouring points to less than %2.3fm. \n', allowableDeviation);
    for j = 1:numberOfTimesPointsAreDoubled
        for i = 1:4
            EEselection = EEnames(i,:);
            [relativeMotionHipEE.(EEselection).position, relativeMotionHipEE.(EEselection).velocity, EE.(EEselection).force, C_IBody] = generateAdditionalPoints(relativeMotionHipEE, EE, C_IBody, EEselection);
         end
        dt = dt/2;
        Leg.time = [0:dt:t(end)]';
    end
    % recompute the deviation from neighbouring point
    for i = 1:4
        EEselection = EEnames(i,:);
        [neighbouringPointDeviation.(EEselection), neighbouringPointDeviationMax.(EEselection)] = getDeviationFromNeighbouringPoint(relativeMotionHipEE, EEselection);
        Leg.metaParameters.neighbouringPointDeviation.(EEselection) = neighbouringPointDeviation.(EEselection);
        Leg.metaParameters.neighbouringPointDeviationMax.(EEselection) = neighbouringPointDeviationMax.(EEselection);
    end
end

%% Get the liftoff and touchdown timings for each end effector
fprintf('Computing end effector liftoff and touchdown timings. \n');
[tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(Leg, EE);

%% Get the mean cyclic position and forces for each end effector
fprintf('Computing average relative motion of end effectors over one step. \n');
[meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd, meanTouchdownIndex] = getHipEECyclicData(dataExtraction, quadruped, tLiftoff, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody, EEnames);
for i = 1:4
    EEselection = EEnames(i,:);
    Leg.(EEselection).force = meanCyclicMotionHipEE.(EEselection).force;
end

%% Get reachable positions for link lengths and joint limits
fprintf('Computing range of motion dependent on link lengths and joint limits. \n');
reachablePositions = getRangeofMotion(quadruped);

%% Plot trajectory data
if viewTrajectoryPlots
    fprintf('Plotting data. \n');
    plotMotionData;
end

%% qAFE, qDFE torque based heuristic computation
% compute the liftoff angle between the final link and the horizontal to
% satisfy the input requirement of thetaLiftoff_des. If this heuristic is
% used, the IK is solved using joint torque from previous timestep to
% define the joint deformation at the current timestep.
if (heuristic.torqueAngle.apply == true) && (linkCount > 2)
    fprintf('Computing joint angles for desired EE liftoff angle. \n');
    for i = 1:4
        EEselection = EEnames(i,:);
        if (EEselection == 'LF') | (EEselection == 'RF')
            hipAttachmentOffset = -quadruped.hip(1).length;
        elseif (EEselection == 'LH') | (EEselection == 'RH')
            hipAttachmentOffset = quadruped.hip(2).length;
        end
        [qLiftoff.(EEselection)] = computeqLiftoffFinalJoint(heuristic, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody, Leg);
        EE_force = Leg.(EEselection).force(1,1:3);
        rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(1,2); % rotation of body about inertial y
        qPrevious = qLiftoff.(EEselection);
    end
    else
        qLiftoff.(EEselection) = 0;
end

%% inverse kinematics
fprintf('Computing joint angles using inverse kinematics.\n');
for i = 1:4
    EEselection = EEnames(i,:);
    if (EEselection == 'LF') | (EEselection == 'RF')
        hipAttachmentOffset = -quadruped.hip(1).length; % initial design keeps hip centered above trajectory but this value can be optimized
    elseif (EEselection == 'LH') | (EEselection == 'RH')
        hipAttachmentOffset = quadruped.hip(2).length;
    end
    [Leg.(EEselection).q, Leg.(EEselection).r.HAA, Leg.(EEselection).r.HFE, Leg.(EEselection).r.KFE, Leg.(EEselection).r.AFE, Leg.(EEselection).r.DFE, Leg.(EEselection).r.EE]  = inverseKinematics(heuristic, qLiftoff, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody);
     Leg.(EEselection).r.EEdes = meanCyclicMotionHipEE.(EEselection).position(1:end,:); % save desired EE position in the same struct for easy comparison
end

%% Build robot rigid body model
fprintf('Creating and visualizing robot rigid body model. \n');
for i = 1:4
    EEselection = EEnames(i,:);
    if (EEselection == 'LF') | (EEselection == 'RF')
        hipAttachmentOffset = -quadruped.hip(1).length;
    elseif (EEselection == 'LH') | (EEselection == 'RH')
        hipAttachmentOffset = quadruped.hip(2).length;
    end
    Leg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(actuatorProperties, actuateJointsDirectly, hipAttachmentOffset, linkCount, quadruped, Leg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody);
end

%% Get joint velocities with finite differences
fprintf('Computing joint velocities and accelerations. \n');
for i = 1:4
    EEselection = EEnames(i,:);
    [Leg.(EEselection).qdot, Leg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(EEselection, Leg);
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
Leg.metaParameters.energyPerCycleTotalOpt = 0;
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
    if imposeJointLimits.maxTorque
        [optimizationProperties.bounds.maxTorqueLimit, ~, ~, ~] = getActuatorProperties(actuatorSelection);
    end
    if imposeJointLimits.maxqdot
        [~, optimizationProperties.bounds.maxqdotLimit, ~, ~] = getActuatorProperties(actuatorSelection);
    end
    if imposeJointLimits.maxPower
        [~, ~, optimizationProperties.bounds.maxPowerLimit, ~] = getActuatorProperties(actuatorSelection);
    end
    Leg.metaParameters.CoTOpt.total = 0;

    if optimizeLF
        EEselection = 'LF';
        hipAttachmentOffset = -quadruped.hipOffset(1);
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
         % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).hipOffsetOpt, Leg.(EEselection).penaltyMinOpt, Leg.metaParameters.elapsedTime.(EEselection), Leg.metaParameters.elapsedTimePerFuncEval.(EEselection), Leg.metaParameters.(EEselection).output, Leg.(EEselection).linkMassOpt, Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg, meanTouchdownIndex);
         % compute CoT 
         power = Leg.(EEselection).jointPowerOpt;
         Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
        [Leg.metaParameters.deltaqMaxOpt.(EEselection), Leg.metaParameters.qdotMaxOpt.(EEselection), Leg.metaParameters.jointTorqueMaxOpt.(EEselection), Leg.metaParameters.jointPowerMaxOpt.(EEselection), Leg.(EEselection).energyOpt, Leg.metaParameters.energyPerCycleOpt.(EEselection)]  = getMaximumJointStates(Leg, power, EEselection, dt);
    end 
    
    if optimizeLH
        EEselection = 'LH';
        hipAttachmentOffset = quadruped.hipOffset(2);        
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).hipOffsetOpt, Leg.(EEselection).penaltyMinOpt, Leg.metaParameters.elapsedTime.(EEselection), Leg.metaParameters.elapsedTimePerFuncEval.(EEselection), Leg.(EEselection).metaParameters.output, Leg.(EEselection).linkMassOpt,  Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg, meanTouchdownIndex);
        % compute CoT
         power = Leg.(EEselection).jointPowerOpt;
         Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
        [Leg.metaParameters.deltaqMaxOpt.(EEselection), Leg.metaParameters.qdotMaxOpt.(EEselection), Leg.metaParameters.jointTorqueMaxOpt.(EEselection), Leg.metaParameters.jointPowerMaxOpt.(EEselection), Leg.(EEselection).energyOpt, Leg.metaParameters.energyPerCycleOpt.(EEselection)]  = getMaximumJointStates(Leg, power, EEselection, dt);         
    end
    
    if optimizeRF
        EEselection = 'RF';
        hipAttachmentOffset = -quadruped.hipOffset(1);      
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).hipOffsetOpt, Leg.(EEselection).penaltyMinOpt, Leg.metaParameters.elapsedTime.(EEselection), Leg.metaParameters.elapsedTimePerFuncEval.(EEselection), Leg.(EEselection).metaParameters.output, Leg.(EEselection).linkMassOpt,  Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg, meanTouchdownIndex);
        % compute CoT
         power = Leg.(EEselection).jointPowerOpt;
         Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
        [Leg.metaParameters.deltaqMaxOpt.(EEselection), Leg.metaParameters.qdotMaxOpt.(EEselection), Leg.metaParameters.jointTorqueMaxOpt.(EEselection), Leg.metaParameters.jointPowerMaxOpt.(EEselection), Leg.(EEselection).energyOpt, Leg.metaParameters.energyPerCycleOpt.(EEselection)]  = getMaximumJointStates(Leg, power, EEselection, dt);
    end
    
    if optimizeRH
        EEselection = 'RH';
        hipAttachmentOffset = quadruped.hipOffset(2);
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).hipOffsetOpt, Leg.(EEselection).penaltyMinOpt, Leg.metaParameters.elapsedTime.(EEselection), Leg.metaParameters.elapsedTimePerFuncEval.(EEselection), Leg.(EEselection).metaParameters.output, Leg.(EEselection).linkMassOpt,  Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg, meanTouchdownIndex);
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