%% select task and robot, configuration and end effector which is to be optimized
taskSelection = 'massivoWalk';
robotSelection = 'massivo';
[removalRatioStart, removalRatioEnd] = getSuggestedRemovalRatios(taskSelection);
load(taskSelection);
dt = t(2) - t(1);
configSelection = 'X';
EEselection = 'LF';

% starting quadruped properties
quadruped = getQuadrupedProperties(robotSelection);
if (EEselection == 'LF') | (EEselection == 'RF')
    selectFrontHind = 1;
else selectFrontHind = 2;
end

%initial guess at link lengths in mm so the values can be integers
initialLinkLengths(1) = round(1000*quadruped.hip(selectFrontHind).length)+2; 
initialLinkLengths(2) = round(1000*quadruped.thigh(selectFrontHind).length);
initialLinkLengths(3) = round(1000*quadruped.shank(selectFrontHind).length);

linkLengths = initialLinkLengths;

opts = optimoptions('ga');
opts.Display = 'iter';
opts.MaxGenerations = 10;
opts.PopulationSize = 10;
% opts.InitialPopulationMatrix = repmat(p0,[5 1]); % Add copies of initial gait
% opts.PlotFcn = @gaplotbestf; % Add progress plot of fitness function
opts.PlotFcn = {@gaplotbestf, @gaplotbestindiv}; % Add progress plot of fitness function

% opts.UseParallel = parallelFlag;

%% Set bounds and constraints
% Upper and lower angle bounds
upperBnd = [1.5*initialLinkLengths(1), ... % Hip limits
            1.5*initialLinkLengths(2), ... % Thigh limits
            1.5*initialLinkLengths(3)];    % Shank limits
            
lowerBnd = [0.5*initialLinkLengths(1), ... % Hip limits
            0.5*initialLinkLengths(2), ... % Thigh limits
            0.5*initialLinkLengths(3)];       % Shank limits

%% Run optimization
costFcn = @(linkLengths)runFastJointTorqueSim(quadruped, linkLengths, selectFrontHind, taskSelection, removalRatioStart, removalRatioEnd, base, quat, t, EE, dt, configSelection, EEselection);
disp(['Running optimization. Population: ' num2str(opts.PopulationSize) ...
      ', Max Generations: ' num2str(opts.MaxGenerations)])
  
%[x, feval] = ga(fun,nvars,A,b,[],[],lb,ub,nonlcon,IntCon, options)
 [linkLengths,penaltyMin] = ga(costFcn,3,[],[],[],[], ... 
                              lowerBnd,upperBnd,[],3,opts);
disp(['Final reward function value: ' num2str(penaltyMin)])


% %% Cleanup
% bdclose(mdlName);
% if parallelFlag
%    delete(gcp('nocreate')); 
% end

%% visualize optimized design
% viewVisualization = 1;
% numberOfLoopRepetitions = 4;
% q.(EEselection).angle = inverseKinematics(meanCyclicMotionHipEE.LF.position, quadruped, EEselection, taskSelection, configSelection);
% [robotConfig, config] = buildRobotRigidBodyModel(quadruped, q, EE, meanCyclicMotionHipEE, EEselection, numberOfLoopRepetitions, viewVisualization);
