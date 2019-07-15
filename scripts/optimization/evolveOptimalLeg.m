function [legDesignParameters, penaltyMin, output] = evolveOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, upperBnd, lowerBnd, actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, initialLinkLengths, taskSelection, robotProperties, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency , actuatorSelection, dataExtraction)
if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
    selectFrontHind = 1;
else 
    selectFrontHind = 2;
end
% Set initial values for link lengths
linkLengths = initialLinkLengths;
legDesignParameters = [linkLengths, hipAttachmentOffset];

% Set optimization options
opts = optimoptions('ga');
opts.Display = 'iter';
opts.MaxGenerations = optimizationProperties.options.maxGenerations;
opts.PopulationSize = optimizationProperties.options.populationSize;
if optimizationProperties.viz.displayBestCurrentLinkLengths
    opts.PlotFcn = {@gaplotbestindiv};
end

%% Run optimization
costFcn = @(legDesignParameters)computePenalty(actuatorProperties, imposeJointLimits, heuristic, legDesignParameters, actuateJointsDirectly, linkCount, optimizationProperties, robotProperties, selectFrontHind, taskSelection, dt, configSelection, EEselection, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction);
disp(['Running optimization. Population: ' num2str(opts.PopulationSize) ...
      ', Max Generations: ' num2str(opts.MaxGenerations)])
for i = 1:length(legDesignParameters)
    nvars(i) = i;
end
 %[x, feval] = ga(fun,nvars,A,b,[],[],lb,ub,nonlcon,IntCon, options)
 [legDesignParameters, penaltyMin, ~, output] = ga(costFcn,length(nvars),[],[],[],[], ... 
                                                 lowerBnd, upperBnd,[],nvars,opts);
disp(['Final penalty function value: ' num2str(penaltyMin)])
end