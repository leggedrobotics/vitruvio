function Leg = runDataExtractionAndOptScripts(viewPlots, viewOptimizedResults, actuatorSelection, dataExtraction, imposeJointLimits, heuristic, actuateJointsDirectly, viewVisualization, numberOfStepsVisualized, linkCount, runOptimization, viewOptimizedLegPlot, optimizeLeg, optimizationProperties, taskSelection, classSelection, configSelection, hipParalleltoBody, legCount)
        
%% display selections
fprintf('Quadruped class: %s.\n', classSelection);
fprintf('Task: %s.\n', taskSelection);
fprintf('Actuator HAA: %s.\n', actuatorSelection.HAA);
fprintf('Actuator HFE: %s.\n', actuatorSelection.HFE);
fprintf('Actuator KFE: %s.\n', actuatorSelection.KFE);
if linkCount > 2
    fprintf('Actuator AFE: %s.\n', actuatorSelection.AFE);
    if linkCount == 4
        fprintf('Actuator DFE: %s.\n', actuatorSelection.DFE);
    end
end
if linkCount == 2
    fprintf('Links: hip, thigh, shank.\n\n')
elseif linkCount == 3
    fprintf('Links: hip, thigh, shank, foot.\n\n')
elseif linkCount == 4
    fprintf('Links: hip, thigh, shank, foot, phalanges.\n\n')
end

%% Load in trajectory

% Load motion and force data from .mat file
load(taskSelection);
dt = t(2) - t(1); % time step is constant 
Leg.time = t;

EEnames = ['LF'; 'RF'; 'LH'; 'RH']; % a subset of these are used depending on legCount
linkNames = {'hip' 'thigh' 'shank' 'foot' 'phalanges'}; % a subset of these are used depending on linkCount
jointNames = ['HAA'; 'HFE'; 'KFE'; 'AFE'; 'DFE']; % a subset of these are used depending on linkCount

% Get suggested removal ratio for cropping motion data to useful steady state motion
[removalRatioStart, removalRatioEnd] = getSuggestedRemovalRatios(taskSelection);

%% Load corresponding robot parameters
% quadruped properties
fprintf('Loading quadruped properties for %s.\n', classSelection);
quadruped = getQuadrupedProperties(classSelection, linkCount);

%actuator properties
fprintf('Loading actuator properties');
for i = 1:linkCount+1
    jointSelection = jointNames(i,:);
    actuatorName = actuatorSelection.(jointSelection);
    [actuatorProperties.maxTorqueLimit.(jointSelection), actuatorProperties.maxqdotLimit.(jointSelection), actuatorProperties.maxPowerLimit.(jointSelection), actuatorProperties.mass.(jointSelection), actuatorProperties.gearRatio.(jointSelection)] = getActuatorProperties(actuatorName);
end

for i = 1:legCount
    EEselection = EEnames(i,:);
     if strcmp(EEselection,'LF') || strcmp(EEselection,'RF')
         selectFrontHind = 1;
     else
         selectFrontHind = 2;
     end
     for j = 1:linkCount+1
        Leg.(EEselection).linkLengths(1,j) = quadruped.(linkNames{j})(selectFrontHind).length;
    end
end

%% Reconstruct terrain map from EE force and position data
terrainMap = constructTerrainMap(EE, legCount, EEnames);

%% Get the relative motion of the end effectors to the hips
fprintf('Computing motion of end effectors relative to hip attachment points. \n');
[relativeMotionHipEE, IF_hip, C_IBody] = getRelativeMotionEEHips(quat, quadruped, base, EE, dt);

%% Get deviation from neighbouring point
setOfNeighbouringPointDeviationMax = [];
for i = 1:legCount
    EEselection = EEnames(i,:);
    [neighbouringPointDeviation.(EEselection), neighbouringPointDeviationMax.(EEselection)] = getDeviationFromNeighbouringPoint(relativeMotionHipEE, EEselection);
    Leg.metaParameters.neighbouringPointDeviation.(EEselection) = neighbouringPointDeviation.(EEselection);
    Leg.metaParameters.neighbouringPointDeviationMax.(EEselection) = neighbouringPointDeviationMax.(EEselection);
    setOfNeighbouringPointDeviationMax = [setOfNeighbouringPointDeviationMax, neighbouringPointDeviationMax.(EEselection)];
end

%% If points are spread too far apart, interpolate to add more points
allowableDeviation = dataExtraction.allowableDeviation;
absoluteMaxPointDeviation = max(setOfNeighbouringPointDeviationMax); % the absolute max deviation for all legs
if absoluteMaxPointDeviation > allowableDeviation
    numberOfTimesPointsAreDoubled = ceil(log(absoluteMaxPointDeviation/allowableDeviation)/log(2)); % because deviation decreases by factor of 2^n
    fprintf('Interpolating points to reduce distance between neighbouring points to less than %2.3fm. \n', allowableDeviation);
    for j = 1:numberOfTimesPointsAreDoubled
        for i = 1:legCount
            EEselection = EEnames(i,:);
            [relativeMotionHipEE.(EEselection).position, relativeMotionHipEE.(EEselection).velocity, EE.(EEselection).force, C_IBody, ~] = generateAdditionalPoints(relativeMotionHipEE, EE, C_IBody, EEselection, base);
        end
        [~, ~, ~, ~, base.position] = generateAdditionalPoints(relativeMotionHipEE, EE, C_IBody, EEselection, base);                         
        dt = dt/2;
        Leg.time = [0:dt:t(end)]';
    end
    % recompute the deviation from neighbouring point
    for i = 1:legCount
        EEselection = EEnames(i,:);
        [neighbouringPointDeviation.(EEselection), neighbouringPointDeviationMax.(EEselection)] = getDeviationFromNeighbouringPoint(relativeMotionHipEE, EEselection);
        Leg.metaParameters.neighbouringPointDeviation.(EEselection) = neighbouringPointDeviation.(EEselection);
        Leg.metaParameters.neighbouringPointDeviationMax.(EEselection) = neighbouringPointDeviationMax.(EEselection);
    end
end

%% Get the liftoff and touchdown timings for each end effector
fprintf('Computing end effector liftoff and touchdown timings. \n');
[tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(Leg, EE, EEnames, legCount);

%% Get the averaged or unaveraged motion and forces as specified by user selections in main
if dataExtraction.averageStepsForCyclicalMotion
    fprintf('Computing average relative motion of end effectors over one step. \n');
    % motion
    [meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd, Leg.base.position] = getHipEECyclicData(tLiftoff, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody, EEnames, base, legCount);    
% get the liftoff and touchdown indices for the new data.


else % do not average the motion, rather use the full motion
    [meanCyclicMotionHipEE, meanCyclicC_IBody, Leg.base.position] = getHipEEFullMotionData(relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, C_IBody, EEnames, base, legCount);
end

% save force data into Leg struct
for i = 1:legCount
    EEselection = EEnames(i,:);
    Leg.(EEselection).force = meanCyclicMotionHipEE.(EEselection).force;
end

%% qAFE, qDFE torque based heuristic computation
% compute the liftoff angle between the final link and the horizontal to
% satisfy the input requirement of thetaLiftoff_des. If this heuristic is
% used, the IK is solved using joint torque from previous timestep to
% define the joint deformation at the current timestep.
if (heuristic.torqueAngle.apply == true) && (linkCount > 2)
    fprintf('Computing joint angles for desired EE liftoff angle. \n');
    for i = 1:legCount
        EEselection = EEnames(i,:);
        if strcmp(EEselection,'LF') || strcmp(EEselection,'RF')
            hipAttachmentOffset = -quadruped.hip(1).length;
        elseif strcmp(EEselection,'LH') || strcmp(EEselection,'RH')
            hipAttachmentOffset = quadruped.hip(2).length;
        end
        % Use inverse kinematics to solve joint angles for first time step.
        [qLiftoff.(EEselection)] = computeqLiftoffFinalJoint(heuristic, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, configSelection, hipParalleltoBody);
        EE_force = Leg.(EEselection).force(1,1:3);
        rotBodyY = -meanCyclicMotionHipEE.body.eulerAngles.(EEselection)(1,2); % rotation of body about inertial y
        qPrevious = qLiftoff.(EEselection);
    end
    else
        qLiftoff.(EEselection) = 0;
end

%% inverse kinematics
fprintf('Computing joint angles using inverse kinematics.\n');
for i = 1:legCount
    EEselection = EEnames(i,:);
    if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
        hipAttachmentOffset = -quadruped.hip(1).length; % initial design keeps hip centered above trajectory but this value can be optimized
    elseif strcmp(EEselection, 'LH') || strcmp(EEselection, 'RH')
        hipAttachmentOffset = quadruped.hip(2).length;
    end
    [Leg.(EEselection).q, Leg.(EEselection).r.HAA, Leg.(EEselection).r.HFE, Leg.(EEselection).r.KFE, Leg.(EEselection).r.AFE, Leg.(EEselection).r.DFE, Leg.(EEselection).r.EE]  = inverseKinematics(heuristic, qLiftoff, hipAttachmentOffset, linkCount, meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection, hipParalleltoBody);
     Leg.(EEselection).r.EEdes = meanCyclicMotionHipEE.(EEselection).position(1:end,:); % save desired EE position in the same struct for easy comparison
end

%% Build robot rigid body model
fprintf('Creating and visualizing robot rigid body model. \n');
for i = 1:legCount
    EEselection = EEnames(i,:);
    if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
        hipAttachmentOffset = -quadruped.hip(1).length;
    elseif strcmp(EEselection, 'LH') || strcmp(EEselection, 'RH')
        hipAttachmentOffset = quadruped.hip(2).length;
    end
    Leg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(actuatorProperties, actuateJointsDirectly, hipAttachmentOffset, linkCount, quadruped, Leg, meanCyclicMotionHipEE, EEselection, numberOfStepsVisualized, viewVisualization, hipParalleltoBody, dataExtraction);
end

%% Get joint velocities with finite differences
fprintf('Computing joint velocities and accelerations. \n');
for i = 1:legCount
    EEselection = EEnames(i,:);
    [Leg.(EEselection).qdot, Leg.(EEselection).qdotdot] = getJointVelocitiesUsingFiniteDifference(EEselection, Leg);
    Leg.(EEselection).q = Leg.(EEselection).q(1:end-2,:); % remove the two supplementary points for position after solving for joint speed and acceleration
end

%% Get joint torques using inverse dynamics
fprintf('Computing joint torques using inverse dynamics. \n');
for i = 1:legCount
    EEselection = EEnames(i,:);
    Leg.(EEselection).jointTorque = inverseDynamics(EEselection, Leg, meanCyclicMotionHipEE, linkCount);
end

%% Get joint power - this is the mechanical power required at the joint
fprintf('Computing joint power. \n');
for i = 1:legCount
    EEselection = EEnames(i,:);
    Leg.(EEselection).jointPower = Leg.(EEselection).jointTorque .* Leg.(EEselection).qdot(:,1:end-1);
end

%% Get efficiency map of actuator
actuatorList = {actuatorSelection.HAA, actuatorSelection.HFE, actuatorSelection.KFE, actuatorSelection.AFE, actuatorSelection.DFE};
actuatorList = unique(actuatorList); % remove duplicates so we only solve the efficiency map once for each type of actuator
for i = 1:length(actuatorList)
    actuatorName = actuatorList(i);
    [actuatorEfficiency.(actuatorName{1}), efficiencyMapPlot.(actuatorName{1})] = getActuatorEfficiencySimplified(actuatorName);
end

%% Get electrical power and efficiency at each operating point
fprintf('Computing electrical power. \n');
for i = 1:legCount
    EEselection = EEnames(i,:);
    [Leg.(EEselection).electricalPower, Leg.(EEselection).operatingPointEfficiency] = computeElectricalPowerInput(Leg, EEselection, actuatorProperties, linkCount, actuatorEfficiency, actuatorSelection);
end

%% Save link lengths and mass
for i = 1:legCount
    EEselection = EEnames(i,:);
    [Leg.(EEselection).linkMass, Leg.(EEselection).EEMass, Leg.(EEselection).totalLinkMass] = getLinkMass(Leg, EEselection, linkCount);
    if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
        Leg.(EEselection).hipOffset = quadruped.hipOffset(1); 
    else
        Leg.(EEselection).hipOffset = quadruped.hipOffset(2);
    end
end
%% Get meta parameters
Leg.CoM.velocity = base.velocity(floor(removalRatioStart*length(base.velocity))+1:floor((1-removalRatioEnd)*length(base.velocity)),:);
Leg.metaParameters.CoT.total = 0; % initialize total cost of transport

for i = 1:legCount
    EEselection = EEnames(i,:);
   
    % cost of transport
    power = Leg.(EEselection).jointPower;
    Leg.metaParameters.CoT.(EEselection) = getCostOfTransport(Leg, power, quadruped);
   
    % add contribution of each leg to get the total CoT
    Leg.metaParameters.CoT.total = Leg.metaParameters.CoT.total + Leg.metaParameters.CoT.(EEselection); 
    
    % power quality
    Leg.metaParameters.(EEselection).powerQuality = computePowerQuality(power, EEselection);

    % max joint states
    [Leg.metaParameters.deltaqMax.(EEselection), Leg.metaParameters.qdotMax.(EEselection), Leg.metaParameters.jointTorqueMax.(EEselection), Leg.metaParameters.jointPowerMax.(EEselection)]  = getMaximumJointStates(Leg, EEselection);
    
    % energy consumption
    [Leg.(EEselection).mechEnergy, Leg.metaParameters.mechEnergyPerCycle.(EEselection), Leg.(EEselection).elecEnergy, Leg.metaParameters.elecEnergyPerCycle.(EEselection)]  = computeEnergyConsumption(Leg, EEselection, dt);
    Leg.metaParameters.mechEnergyPerCycleTotal.(EEselection) = sum(Leg.metaParameters.mechEnergyPerCycle.(EEselection));
    Leg.metaParameters.elecEnergyPerCycleTotal.(EEselection) = sum(Leg.metaParameters.elecEnergyPerCycle.(EEselection));    
   
    % phase durations
    [Leg.metaParameters.swingDuration.(EEselection), Leg.metaParameters.stanceDuration.(EEselection), Leg.metaParameters.swingDurationRatio.(EEselection)] = computePhaseDurations(tLiftoff, tTouchdown, EEselection);
end

%% Optimize selected legs and compute their cost of transport
if runOptimization % master toggle in main
    % Get actuator limits which are enforced by the optimizer
    for i = 1:linkCount+1
        jointSelection = jointNames(i,:);
        actuatorName = actuatorSelection.(jointSelection);
        if imposeJointLimits.maxTorque || imposeJointLimits.maxqdot || imposeJointLimits.maxPower
            [optimizationProperties.bounds.maxTorqueLimit.(jointSelection), optimizationProperties.bounds.maxqdotLimit.(jointSelection), optimizationProperties.bounds.maxPowerLimit.(jointSelection), ~, ~] = getActuatorProperties(actuatorName);
        end
    end
    
    %% Optimize the legs selected for optimization in main
    Leg.metaParameters.CoTOpt.total = 0;
    for i = 1:legCount
            EEselection = EEnames(i,:);
            if strcmp(EEselection,'LF') || strcmp(EEselection,'RF')
                selectFrontHind = 1;
            else
                selectFrontHind = 2;
            end
            
        % run optimization    
        if optimizeLeg.(EEselection) % toggle in main to select legs for optimization
            hipAttachmentOffset = -quadruped.hipOffset(selectFrontHind);
            fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
             % evolve leg and return joint data of optimized design
            [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt, Leg.(EEselection).rOpt,  Leg.(EEselection).jointPowerOpt, Leg.(EEselection).linkLengthsOpt, Leg.(EEselection).hipOffsetOpt, Leg.(EEselection).penaltyMinOpt, Leg.metaParameters.elapsedTime.(EEselection), Leg.metaParameters.elapsedTimePerFuncEval.(EEselection), Leg.metaParameters.(EEselection).output, Leg.(EEselection).linkMassOpt, Leg.(EEselection).totalLinkMassOpt, Leg.metaParameters.deltaqMaxOpt.(EEselection), Leg.metaParameters.qdotMaxOpt.(EEselection), Leg.metaParameters.jointTorqueMaxOpt.(EEselection), Leg.metaParameters.jointPowerMaxOpt.(EEselection), Leg.(EEselection).mechEnergyOpt, Leg.metaParameters.mechEnergyPerCycleOpt.(EEselection), Leg.metaParameters.mechEnergyPerCycleTotalOpt.(EEselection), Leg.(EEselection).elecEnergyOpt, Leg.metaParameters.elecEnergyPerCycleOpt.(EEselection), Leg.metaParameters.elecEnergyPerCycleTotalOpt.(EEselection)] = evolveAndVisualizeOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, configSelection, dt, taskSelection, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction);
             % compute CoT
             power = Leg.(EEselection).jointPowerOpt;
             Leg.metaParameters.CoTOpt.(EEselection) = getCostOfTransport(Leg, power, quadruped);
             Leg.metaParameters.powerQualityOpt.(EEselection) = computePowerQuality(power, EEselection);
        end
    end
end
 
%% Generate plots
% Plot trajectory data
if viewPlots.trajectoryPlots
    fprintf('Plotting data. \n');
    plotMotionData;
end

% Plot range of motion
if viewPlots.rangeOfMotionPlots
    fprintf('Computing range of motion dependent on link lengths and joint limits. \n');
    plotRangeOfMotion(quadruped, Leg, cyclicMotionHipEE);
end

% Plot actuator operating points on efficiency map
if viewPlots.efficiencyMap
    viewOptimizedData = false; % there is no optimized data at this point
    plotEfficiencyMapWithOperatingPoints(viewOptimizedData, [], EEnames, actuatorSelection, Leg, efficiencyMapPlot, actuatorProperties, linkCount, meanTouchdownIndex, dataExtraction)
end

% Plot joint data (q, qdot, torque, power, energy). Optionally with
% optimized result on the same axes. As scatter or line plot. Optionally with 
% another set of data on the same axes.
plotJointDataForAllLegs(ANYmal, 'slowTrot', [], [], plotOptimizedLeg, 'scatterPlot');

% plot results of optimization
if runOptimization % master toggle in main
        if viewOptimizedLegPlot % master toggle for all optimized leg plots
            fprintf('Plotting initial and optimized leg designs \n');
            viewOptimizedData = true;
            for i = 1:legCount
            EEselection = EEnames(i,:);
                if viewOptimizedResults.jointDataPlot
                    fprintf('Plotting joint data \n');                   
                    plotOptimizedJointTorque(Leg, EEselection, dt, meanTouchdownIndex, taskSelection, linkCount)
                end
                if viewOptimizedResults.metaParameterPlot
                    fprintf('Plotting meta parameters \n');                                       
                    plotMetaParameters(Leg, linkCount, EEselection);
                end
                if viewOptimizedResults.efficiencyMap
                    fprintf('Plotting operating points and efficiency map \n');                                                          
                    plotEfficiencyMapWithOperatingPoints(viewOptimizedData, EEselection, EEnames, actuatorSelection, Leg, efficiencyMapPlot, actuatorProperties, linkCount, meanTouchdownIndex, dataExtraction)
                end
            end
        end
end
end