function [legDesignParameters, penaltyMin, output] = evolveOptimalLeg(imposeJointLimits, heuristic, upperBnd, lowerBnd, actuateJointsDirectly, hipAttachmentOffset, linkCount, optimizationProperties, initialLinkLengths, taskSelection, quadruped, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg, meanTouchdownIndex)
if (EEselection == 'LF') | (EEselection == 'RF')
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
costFcn = @(legDesignParameters)computePenalty(imposeJointLimits, heuristic, legDesignParameters, actuateJointsDirectly, linkCount, optimizationProperties, quadruped, selectFrontHind, taskSelection, dt, configSelection, EEselection, meanCyclicMotionHipEE, hipParalleltoBody, Leg, meanTouchdownIndex);
disp(['Running optimization. Population: ' num2str(opts.PopulationSize) ...
      ', Max Generations: ' num2str(opts.MaxGenerations)])
for i = 1:length(legDesignParameters)
    nvars(i) = i;
end
 %[x, feval] = ga(fun,nvars,A,b,[],[],lb,ub,nonlcon,IntCon, options)
 [legDesignParameters, penaltyMin, ~, output] = ga(costFcn,length(nvars),[],[],[],[], ... 
                                                 lowerBnd*100,upperBnd*100,[],nvars,opts);
disp(['Final penalty function value: ' num2str(penaltyMin)])

end