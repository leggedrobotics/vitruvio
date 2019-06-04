function Leg = runDataExtractionAndOptScripts(viewVisualization, numberOfLoopRepetitions, viewTrajectoryPlots, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLF, optimizeLH, optimizeRF, optimizeRH, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody)

EEnames = ['LF'; 'RF'; 'LH'; 'RH'];
linkNames = {'hip','thigh' 'shank' 'foot' 'phalanges'};
l_hipAttachmentOffset = 0; % this will become an optimization parameter but for now is hard coded
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
fprintf('Computing motion of end effectors relative to hip attachment points \n');
[relativeMotionHipEE, IF_hip, C_IBody] = getRelativeMotionEEHips(quat, quadruped, base, EE, dt);

%% Get the liftoff and touchdown timings for each end effector
dt = t(2) - t(1);
fprintf('Computing end effector liftoff and touchdown timings \n');
[tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(t, EE);

%% Get the mean cyclic position and forces for each end effector
fprintf('Computing average relative motion of end effectors over one step \n');
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
fprintf('Computing range of motion dependent on link lengths and joint limits \n');
reachablePositions = getRangeofMotion(quadruped);

%% Plot trajectory data
if viewTrajectoryPlots
    fprintf('Plotting data. \n');
    plotMotionData;
end

%% Inverse kinematics to calculate joint angles for each leg joint
% compute qAFE for 3 link leg
if linkCount == 3
    for i = 1:4
        EEselection = EEnames(i,:);
        Leg.(EEselection).q(:,4) = computeAFE(Leg, EEselection, configSelection);
    end
end

fprintf('Computing joint angles using inverse kinematics. \n');
for i = 1:4
    EEselection = EEnames(i,:);
    [Leg.(EEselection).q, Leg.(EEselection).r.HAA, Leg.(EEselection).r.HFE, Leg.(EEselection).r.KFE, Leg.(EEselection).r.AFE, Leg.(EEselection).r.DFE, Leg.(EEselection).r.EE]  = inverseKinematics(l_hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody, Leg);
     Leg.(EEselection).r.EEdes = meanCyclicMotionHipEE.(EEselection).position; % save desired EE position in the same struct for easy comparison
end

%% Build robot rigid body model
fprintf('Creating robot rigid body model. \n');
for i = 1:4
    EEselection = EEnames(i,:);
    Leg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(l_hipAttachmentOffset, linkCount, quadruped, Leg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization, hipParalleltoBody);
end

%% Get joint velocities with inverse(Jacobian)* EE.velocity
% the joint accelerations are then computed using finite difference
fprintf('Computing joint velocities and accelerations. \n');
for i = 1:4
    EEselection = EEnames(i,:);
    [Leg.(EEselection).qdot, Leg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(linkCount, EEselection, meanCyclicMotionHipEE, Leg, quadruped, dt);
end

%% Get joint torques using inverse dynamics
fprintf('Computing joint torques using inverse dynamics \n');
for i = 1:4
    EEselection = EEnames(i,:);
    Leg.(EEselection).jointTorque = inverseDynamics(EEselection, Leg, meanCyclicMotionHipEE, linkCount);
end

%% Get joint power
fprintf('Computing joint power \n \n');
for i = 1:4
    EEselection = EEnames(i,:);
    Leg.(EEselection).jointPower = Leg.(EEselection).jointTorque .* Leg.(EEselection).qdot(1:end-2,1:end-1);
end

%% Get leg mass

for i = 1:4
    EEselection = EEnames(i,:);
    [Leg.(EEselection).linkMass, Leg.(EEselection).EEMass, Leg.(EEselection).totalLinkMass] = getLinkMass(Leg, EEselection, linkCount);
end

%% Get cost of transport
Leg.CoM.velocity = base.velocity(floor(removalRatioStart*length(base.velocity))+1:floor((1-removalRatioEnd)*length(base.velocity)),:);
Leg.metaParameters.CoT.total = 0;
for i = 1:4
    EEselection = EEnames(i,:);
    power = Leg.(EEselection).jointPower;
    Leg.metaParameters.CoT.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
    % add contribution of each leg to get the total CoT
    Leg.metaParameters.CoT.total = Leg.metaParameters.CoT.total + Leg.metaParameters.CoT.(EEselection); 
end
%% Optimize selected legs and compute their cost of transport
if runOptimization
    Leg.metaParameters.CoTOpt.total = 0;
    if optimizeLF
        EEselection = 'LF';
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
         % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).penaltyMin, Leg.(EEselection).elapsedTime, Leg.(EEselection).elapsedTimePerFuncEval, Leg.(EEselection).output, Leg.(EEselection).linkMassOpt, Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(l_hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg);
         % compute CoT 
         power = Leg.(EEselection).jointPowerOpt;
         Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
    end 
    if optimizeLH
        EEselection = 'LH';
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).penaltyMin, Leg.(EEselection).elapsedTime, Leg.(EEselection).elapsedTimePerFuncEval, Leg.(EEselection).output, Leg.(EEselection).linkMassOpt,  Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(l_hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg);
        % compute CoT
         power = Leg.(EEselection).jointPowerOpt;
         Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
    end
    if optimizeRF
        EEselection = 'RF';
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).penaltyMin, Leg.(EEselection).elapsedTime, Leg.(EEselection).elapsedTimePerFuncEval, Leg.(EEselection).output, Leg.(EEselection).linkMassOpt,  Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(l_hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg);
        % compute CoT
         power = Leg.(EEselection).jointPowerOpt;
         Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
    end
    if optimizeRH
        EEselection = 'RH';
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        % evolve leg
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).penaltyMin, Leg.(EEselection).elapsedTime, Leg.(EEselection).elapsedTimePerFuncEval, Leg.(EEselection).output, Leg.(EEselection).linkMassOpt,  Leg.(EEselection).totalLinkMassOpt] = evolveAndVisualizeOptimalLeg(l_hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg);
        % compute CoT
         power = Leg.(EEselection).jointPowerOpt;
         Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped, EEselection);
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
