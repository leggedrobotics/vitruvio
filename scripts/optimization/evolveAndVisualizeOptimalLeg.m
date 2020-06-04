% this function calls evolveOptimalLeg which starts the optimization by calling computePenalty 
function optimizationResults = evolveAndVisualizeOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, actuateJointDirectly, linkCount, optimizationProperties, EEselection, meanCyclicMotionHipEE, robotProperties, configSelection, dt, taskSelection, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction, jointNames, saveFiguresToPDF, springInParallelWithJoints, kSpringJoint, q0SpringJoint)
if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
    selectFrontHind = 1;
else 
    selectFrontHind = 2;
end
%% Initialize link length values
initialLinkLengths(1) = robotProperties.hip(selectFrontHind).length; 
initialLinkLengths(2) = robotProperties.thigh(selectFrontHind).length;
initialLinkLengths(3) = robotProperties.shank(selectFrontHind).length;
if (linkCount == 3) || (linkCount == 4)
    initialLinkLengths(4) = robotProperties.foot(selectFrontHind).length;
end
if (linkCount == 4)
    initialLinkLengths(5) = robotProperties.phalanges(selectFrontHind).length;
end

%% Input the upper and lower bounds for the optimization
% Order of bounds is as follows:
% [link lengths, hip offset x from nom position, transmission gear ratios,
% torsional spring constant, liftoff angle]

upperBound = [optimizationProperties.bounds.upperBoundMultiplier.hipLength*initialLinkLengths(1), ...
              optimizationProperties.bounds.upperBoundMultiplier.thighLength*initialLinkLengths(2), ...
              optimizationProperties.bounds.upperBoundMultiplier.shankLength*initialLinkLengths(3)];

lowerBound = [optimizationProperties.bounds.lowerBoundMultiplier.hipLength*initialLinkLengths(1), ...
              optimizationProperties.bounds.lowerBoundMultiplier.thighLength*initialLinkLengths(2), ...
              optimizationProperties.bounds.lowerBoundMultiplier.shankLength*initialLinkLengths(3)];  
          
if linkCount == 2 % hip attachment offset
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.KFE];
              
    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.KFE];
end
          
if linkCount == 3 % foot length and hip attachment offset
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.footLength.*initialLinkLengths(4), ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.KFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.AFE];
              
    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.footLength*initialLinkLengths(4), ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.KFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.AFE];    
end

if linkCount == 4 % phalanges length and hip attachment offset
    % Bounds on phalanges link length
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.footLength.*initialLinkLengths(4), ...    
                  optimizationProperties.bounds.upperBoundMultiplier.phalangesLength.*initialLinkLengths(5), ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.KFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.AFE, ...
                  optimizationProperties.bounds.upperBoundMultiplier.transmissionGearRatio.DFE];

    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.footLength.*initialLinkLengths(4), ...
                  optimizationProperties.bounds.lowerBoundMultiplier.phalangesLength.*initialLinkLengths(5), ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HAA, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.HFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.KFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.AFE, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.transmissionGearRatio.DFE];
end


              
if linkCount > 2 && heuristic.torqueAngle.apply % Bounds on torsional spring at final joint
    upperBound = [upperBound, ...
                  optimizationProperties.bounds.upperBoundMultiplier.kTorsionalSpring.*heuristic.torqueAngle.kTorsionalSpring, ...
                  optimizationProperties.bounds.upperBoundMultiplier.thetaLiftoff_des.*heuristic.torqueAngle.thetaLiftoff_des];
    
    lowerBound = [lowerBound, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.kTorsionalSpring.*heuristic.torqueAngle.kTorsionalSpring, ...
                  optimizationProperties.bounds.lowerBoundMultiplier.thetaLiftoff_des.*heuristic.torqueAngle.thetaLiftoff_des];  
end

% Extend end of bounds array with the bounds on spring constants for
% springs in parallel with the joints
if springInParallelWithJoints
    for i = 1:linkCount+1  
        upperBound(end+1) = optimizationProperties.bounds.upperBoundMultiplier.kSpringJoint.(jointNames(i,:))*kSpringJoint.(EEselection)(i);
        lowerBound(end+1) = optimizationProperties.bounds.lowerBoundMultiplier.kSpringJoint.(jointNames(i,:))*kSpringJoint.(EEselection)(i);
    end
end

% Ensure bounds ordered corrrectly such that lower bound <= upper bound.
for i = 1:length(upperBound)
    if (lowerBound(i) > upperBound(i))
        lowerBoundTemp(i) = lowerBound(i);
        lowerBound(i) = upperBound(i);
        upperBound(i) = lowerBoundTemp(i);
    end
end

% Get design parameter names
if linkCount == 2 % hip attachment offset
    designParameterNames = {'hip length', ...
                            'thigh length', ...
                            'shank length', ...
                            'HAA transmission gear ratio', ...
                            'HFE transmission gear ratio', ...
                            'KFE transmission gear ratio'};
                        
elseif linkCount == 3
    designParameterNames = {'hip length', ...
                            'thigh length', ...
                            'shank length', ...
                            'foot length', ...
                            'HAA transmission gear ratio', ...
                            'HFE transmission gear ratio', ...
                            'KFE transmission gear ratio', ... 
                            'AFE transmission gear ratio'};
                        
elseif linkCount == 4 
    designParameterNames = {'hip length', ...
                            'thigh length', ...
                            'shank length', ...
                            'foot length', ...
                            'phalanges length', ...
                            'HAA transmission gear ratio', ...
                            'HFE transmission gear ratio', ...
                            'KFE transmission gear ratio', ... 
                            'AFE transmission gear ratio', ...
                            'DFE transmission gear ratio'};
end

if springInParallelWithJoints
    designParameterNames = [designParameterNames, ...
                            {'torsional spring constant at HAA'}, ...
                            {'torsional spring constant at HFE'}, ...
                            {'torsional spring constant at KFE'}];
    if linkCount > 2
        designParameterNames = [designParameterNames, ...
                                {'torsional spring constant at AFE'}]; 
    end
    if linkCount > 3
        designParameterNames = [designParameterNames, ...
                                {'torsional spring constant at DFE'}]; 
    end
end
        

if linkCount > 2 && heuristic.torqueAngle.apply
    designParameterNames = [designParameterNames, ...
                            {'torsional spring constant AFE/DFE', ...
                            'liftoff angle'}];
end

for i = 1:length(designParameterNames)
    fprintf('Bounds on %s [%3.2f, %3.2f] \n', designParameterNames{i}, lowerBound(i), upperBound(i))
end

%% Evolve optimal leg design and return optimized design parameters
tic;
[legDesignParameters, penaltyMin, output] = evolveOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, upperBound, lowerBound, actuateJointDirectly, linkCount, optimizationProperties, initialLinkLengths, taskSelection, robotProperties, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction, jointNames, springInParallelWithJoints, kSpringJoint, q0SpringJoint);
optimizationResults.elapsedTime = toc;
optimizationResults.elapsedTimePerFuncEval = optimizationResults.elapsedTime/output.funccount;
fprintf('Optimized leg design parameters :')
disp(legDesignParameters);

[~, tempLeg] = computePenalty(actuatorProperties, imposeJointLimits, heuristic, legDesignParameters, actuateJointDirectly, linkCount, optimizationProperties, robotProperties, selectFrontHind, taskSelection, dt, configSelection, EEselection, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction, springInParallelWithJoints, kSpringJoint, q0SpringJoint);

%% Get maximum joint states
[deltaqMaxOpt, qdotMaxOpt, jointTorqueMaxOpt, jointPowerMaxOpt]  = getMaximumJointStates(tempLeg, EEselection);    
% Maximum actuator states
for j = 1:linkCount+1
    actuatordeltaqMaxOpt = tempLeg.(EEselection).transmissionGearRatio .* deltaqMaxOpt;
    actuatorqdotMaxOpt   = tempLeg.(EEselection).transmissionGearRatio .* qdotMaxOpt;
    actuatorTorqueMaxOpt = (1./tempLeg.(EEselection).transmissionGearRatio) .* jointTorqueMaxOpt;
end
% 
% %% Get electrical power and efficiency at each operating point
% [tempLeg.(EEselection).electricalPower, tempLeg.(EEselection).operatingPointEfficiency] = computeElectricalPowerInput(tempLeg, EEselection, actuatorProperties, linkCount, actuatorEfficiency, actuatorSelection);

%% Get meta parameters
%[tempLeg.(EEselection).mechEnergy, tempLeg.metaParameters.mechEnergyPerCycle.(EEselection), tempLeg.(EEselection).elecEnergy, tempLeg.metaParameters.elecEnergyPerCycle.(EEselection)]  = computeEnergyConsumption(tempLeg.(EEselection).jointPower, tempLeg.(EEselection).electricalPower, dt);
tempLeg.metaParameters.mechEnergyPerCycleTotal.(EEselection) = sum(tempLeg.metaParameters.mechEnergyPerCycle.(EEselection)(end,:));
tempLeg.metaParameters.elecEnergyPerCycleTotal.(EEselection) = sum(tempLeg.metaParameters.elecEnergyPerCycle.(EEselection)(end,:));  
[mechEnergyActiveOpt, mechEnergyPerCycleActiveOpt, ~, ~]  = computeEnergyConsumption(tempLeg.(EEselection).activePower, tempLeg.(EEselection).electricalPower, dt);

%% Return the results of the optimization
optimizationResults.rigidBodyModelSwingOpt          = tempLeg.(EEselection).rigidBodyModelSwing;
optimizationResults.linkLengthsOpt                  = tempLeg.(EEselection).linkLengths;
optimizationResults.transmissionGearRatioOpt        = tempLeg.(EEselection).transmissionGearRatio;
optimizationResults.springOpt.kTorsionalSpring      = tempLeg.(EEselection).kTorsionalSpring;
optimizationResults.springOpt.thetaLiftoff_des      = tempLeg.(EEselection).thetaLiftoff_des;
optimizationResults.rOpt                            = tempLeg.(EEselection).r;
optimizationResults.qOpt                            = tempLeg.(EEselection).q;
optimizationResults.qdotOpt                         = tempLeg.(EEselection).qdot;
optimizationResults.qdotdotOpt                      = tempLeg.(EEselection).qdotdot;
optimizationResults.jointTorqueOpt                  = tempLeg.(EEselection).jointTorque;
optimizationResults.jointPowerOpt                   = tempLeg.(EEselection).jointPower;
optimizationResults.mechEnergyOpt                   = tempLeg.(EEselection).mechEnergy;
optimizationResults.mechEnergyPerCycleOpt           = tempLeg.metaParameters.mechEnergyPerCycle.(EEselection);
optimizationResults.mechEnergyPerCycleTotalOpt      = tempLeg.metaParameters.mechEnergyPerCycleTotal.(EEselection);
optimizationResults.elecEnergyOpt                   = tempLeg.(EEselection).elecEnergy;
optimizationResults.elecEnergyPerCycleOpt           = tempLeg.metaParameters.elecEnergyPerCycle.(EEselection);
optimizationResults.elecEnergyPerCycleTotalOpt      = tempLeg.metaParameters.elecEnergyPerCycleTotal.(EEselection);
optimizationResults.elecPowerOpt                    = tempLeg.(EEselection).electricalPower;
optimizationResults.operatingPointEfficiencyOpt     = tempLeg.(EEselection).operatingPointEfficiency;
optimizationResults.operatingPointEfficiencyMeanOpt = mean(tempLeg.(EEselection).operatingPointEfficiency);
optimizationResults.actuatorqOpt                    = tempLeg.(EEselection).actuatorq;
optimizationResults.actuatorqdotOpt                 = tempLeg.(EEselection).actuatorqdot;
optimizationResults.actuatorTorqueOpt               = tempLeg.(EEselection).actuatorTorque;
optimizationResults.penaltyMinOpt                   = penaltyMin;
optimizationResults.linkMassOpt                     = tempLeg.linkMass;
optimizationResults.totalLinkMassOpt                = sum(tempLeg.linkMass);
optimizationResults.deltaqMaxOpt                    = deltaqMaxOpt;
optimizationResults.qdotMaxOpt                      = qdotMaxOpt;
optimizationResults.jointTorqueMaxOpt               = jointTorqueMaxOpt;
optimizationResults.jointPowerMaxOpt                = jointPowerMaxOpt;
optimizationResults.actuatordeltaqMaxOpt            = actuatordeltaqMaxOpt;
optimizationResults.actuatorqdotMaxOpt              = actuatorqdotMaxOpt;
optimizationResults.actuatorTorqueMaxOpt            = actuatorTorqueMaxOpt;
optimizationResults.gaSettings                      = output;
optimizationResults.activeTorqueOpt                 = tempLeg.(EEselection).activeTorque;
optimizationResults.activePowerOpt                  = tempLeg.(EEselection).activePower;
optimizationResults.passiveTorqueOpt                = tempLeg.(EEselection).passiveTorque;
optimizationResults.passivePowerOpt                 = tempLeg.(EEselection).passivePower;
optimizationResults.kSpringJointOpt                 = tempLeg.(EEselection).kSpringJoint;
optimizationResults.mechEnergyActiveOpt             = mechEnergyActiveOpt;
optimizationResults.mechEnergyPerCycleActiveOpt     = mechEnergyPerCycleActiveOpt;