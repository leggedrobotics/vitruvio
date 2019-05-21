clear;
close all;

%% Toggle visualization and optimization functions
% Toggle trajectory data plots and initial leg design visualization 
viewVisualization = true; 
numberOfLoopRepetitions = 1;
viewTrajectoryPlots = false;

% Toggle optimization and set optimization properties
runOptimization = false;
viewOptimizedLegPlot = true;
optimizeLF = true; 
optimizeLH = false; 
optimizeRF = false; 
optimizeRH = false;
optimizationProperties.viewVisualization = true;
optimizationProperties.displayBestCurrentLinkLengths = false; % display chart while running ga
optimizationProperties.upperBoundMultiplier = [1, 3, 3]; % [hip thigh shank]
optimizationProperties.lowerBoundMultiplier = [1, 0.5, 0.5]; % [hip thigh shank]
optimizationProperties.maxGenerations = 10;
optimizationProperties.populationSize = 10;
optimizationProperties.penaltyWeight.totalTorque = 50;
optimizationProperties.penaltyWeight.totalqdot = 0;
optimizationProperties.penaltyWeight.totalPower = 0;
optimizationProperties.penaltyWeight.maxTorque = 0;
optimizationProperties.penaltyWeight.maxqdotPower = 0;
optimizationProperties.penaltyWeight.maxPower = 0;
optimizationProperties.penaltyWeight.trackingError = 1000;
    
%% Load task
% Select task and robot to be loaded
taskSelection = 'speedyGallop'; % universalTrot, universalStairs, speedyGallop, speedyStairs, massivoWalk, massivoStairs, centaurWalk, centaurStairs, miniPronk
robotSelection = 'speedy'; %universal, speedy, mini, massivo, centaur
configSelection = 'M'; % X, M

EEnames = ['LF'; 'RF'; 'LH'; 'RH'];
fprintf('Loading data for task %s.\n', taskSelection);

% Get suggested removal ratio for cropping motion data to useful steady state motion
[removalRatioStart, removalRatioEnd] = getSuggestedRemovalRatios(taskSelection);

% Load motion and force data from .mat file
load(taskSelection);

%% Load corresponding robot parameters
fprintf('Loading quadruped properties for %s.\n', robotSelection);
quadruped = getQuadrupedProperties(robotSelection);
quadruped.thigh(1).length = 0.35; 
quadruped.shank(1).length = 0.52;


%% Get the relative motion of the end effectors to the hips
fprintf('Computing motion of end effectors relative to hip attachment points \n');
[relativeMotionHipEE, IF_hip, C_IBody] = getRelativeMotionEEHips(quat, quadruped, base, EE, dt);

%% Get the liftoff and touchdown timings for each end effector
dt = t(2) - t(1);
fprintf('Computing end effector liftoff and touchdown timings \n');
[tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(t, EE);

%% Get the mean cyclic position and forces for each end effector
fprintf('Computing average relative motion of end effectors over one step \n');
[meanEuler, meanCyclicMotionHipEE, cyclicMotionHipEE, meanCyclicC_IBody, samplingStart, samplingEnd] = getHipEECyclicData(quadruped, tLiftoff, relativeMotionHipEE, EE, removalRatioStart, removalRatioEnd, dt, minStepCount, C_IBody);

%% Get reachable positions for link lengths and joint limits
fprintf('Computing range of motion dependent on link lengths and joint limits \n');
reachablePositions = getRangeofMotion(quadruped);

%% Plot trajectory data
if viewTrajectoryPlots
    fprintf('Plotting data \n');
    plotMotionData;
end

%% Inverse kinematics to calculate joint angles for each leg joint
fprintf('Computing joint angles using inverse kinematics \n');
for i = 1:4
    EEselection = EEnames(i,:);
    Leg.(EEselection).q = inverseKinematics(meanCyclicMotionHipEE, quadruped, EEselection, taskSelection, configSelection);
end

%% Forward kinematics to get joint positions relative to hip attachment point
fprintf('Computing joint positions relative to the hip attachment point using forward kinematics \n');
jointCount = 4; % [HAA HFE KFE EE] not yet able to handle AFE joint
for i = 1:4
    EEselection = EEnames(i,:);
    Leg.(EEselection).r = getJointPositions(quadruped, Leg, jointCount, EEselection);
end

%% Build robot rigid body model
fprintf('Creating robot rigid body model \n');
for i = 1:4
    EEselection = EEnames(i,:);
    Leg.(EEselection).rigidBodyModel = buildRobotRigidBodyModel(quadruped, Leg, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);
end

%% Get joint velocities with inverse(Jacobian)* EE.velocity
% the joint accelerations are then computed using finite difference
fprintf('Computing joint velocities and accelerations \n');
for i = 1:4
    EEselection = EEnames(i,:);
    [Leg.(EEselection).qdot, Leg.(EEselection).qdotdot] = getJointVelocitiesUsingJacobian(EEselection, meanCyclicMotionHipEE, Leg, quadruped, dt);
end

%% Get joint torques using inverse dynamics
fprintf('Computing joint torques using inverse dynamics \n');
for i = 1:4
    EEselection = EEnames(i,:);
    Leg.(EEselection).jointTorque = getInverseDynamics(EEselection, Leg, meanCyclicMotionHipEE);
end

%% Optimize selected legs
if runOptimization
    if optimizeLF
        EEselection = 'LF';
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt] = evolveAndVisualizeOptimalLeg(optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, jointCount, configSelection, dt, taskSelection, EE);
    end 
    if optimizeLH
        EEselection = 'LH';
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt] = evolveAndVisualizeOptimalLeg(optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, jointCount, configSelection, dt, taskSelection, EE);
    end
    if optimizeRF
        EEselection = 'RF';
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt] = evolveAndVisualizeOptimalLeg(optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, jointCount, configSelection, dt, taskSelection, EE);
    end
    if optimizeRH
        EEselection = 'RH';
        fprintf('\nInitiating optimization of link lengths for %s\n', EEselection);
        [Leg.(EEselection).jointTorqueOpt, Leg.(EEselection).qdotOpt, Leg.(EEselection).qdotdotOpt] = evolveAndVisualizeOptimalLeg(optimizationProperties, EEselection, meanCyclicMotionHipEE, quadruped, jointCount, configSelection, dt, taskSelection, EE);
    end  
%% plot joint torque and speed for initial and optimized design
    if viewOptimizedLegPlot
        fprintf('Plotting joint data for initial and optimized leg designs \n');
        if optimizeLF
            EEselection = 'LF';
            plotOptimizedJointTorque(Leg, EEselection, dt)
        end
        if optimizeLH
            EEselection = 'LH';
            plotOptimizedJointTorque(Leg, EEselection, dt)
        end
        if optimizeRF
            EEselection = 'RF';
            plotOptimizedJointTorque(Leg, EEselection, dt)
        end
        if optimizeRH
            EEselection = 'RH';
            plotOptimizedJointTorque(Leg, EEselection, dt)
        end
    end
end
