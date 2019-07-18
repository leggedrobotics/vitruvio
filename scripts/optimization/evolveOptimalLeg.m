function [legDesignParameters, penaltyMin, output] = evolveOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, upperBound, lowerBound, actuateJointDirectly, hipAttachmentOffset, linkCount, optimizationProperties, initialLinkLengths, taskSelection, robotProperties, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency , actuatorSelection, dataExtraction)
if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
    selectFrontHind = 1;
else 
    selectFrontHind = 2;
end
% Set initial values for link lengths
linkLengths = initialLinkLengths;
legDesignParameters = [linkLengths, hipAttachmentOffset];
% If also optimizing spring parameters
if linkCount>2 && heuristic.torqueAngle.apply
    kTorsionalSpring = heuristic.torqueAngle.kTorsionalSpring;
    thetaLiftoff_des = heuristic.torqueAngle.thetaLiftoff_des;
    legDesignParameters = [linkLengths, hipAttachmentOffset, kTorsionalSpring, thetaLiftoff_des];
end

% Set optimization options
opts = optimoptions('ga');
opts.Display = 'iter';
opts.MaxGenerations = optimizationProperties.options.maxGenerations;
opts.PopulationSize = optimizationProperties.options.populationSize;
if optimizationProperties.viz.displayBestCurrentDesign
    opts.PlotFcn = {@gaplotbestindiv};
end

%% Run optimization
% Compute penalty runs the simulation using legDesignParameters to obtain 
% joint data and evaluates the penalty function for the simulated result. 
costFcn = @(legDesignParameters)computePenalty(actuatorProperties, imposeJointLimits, heuristic, legDesignParameters, actuateJointDirectly, linkCount, optimizationProperties, robotProperties, selectFrontHind, taskSelection, dt, configSelection, EEselection, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction);
disp(['Running optimization. Population: ' num2str(opts.PopulationSize) ...
      ', Max Generations: ' num2str(opts.MaxGenerations)])
for i = 1:length(legDesignParameters)
    nvars(i) = i;
end
 %[x, feval] = ga(fun,nvars,A,b,[],[],lb,ub,nonlcon,intcon, options) %
 % intcon: <Index vector for integer variables>
 [legDesignParameters, penaltyMin, ~, output] = ga(costFcn,length(legDesignParameters),[],[],[],[], ... 
                                                 lowerBound, upperBound,[],[],opts);
disp(['Final penalty function value: ' num2str(penaltyMin)])
end