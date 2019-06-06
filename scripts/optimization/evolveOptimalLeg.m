function [linkLengths, penaltyMin, output] = evolveOptimalLeg(actuateJointsDirectly, l_hipAttachmentOffset, linkCount, optimizationProperties, initialLinkLengths, taskSelection, quadruped, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg, meanTouchdownIndex)
if (EEselection == 'LF') | (EEselection == 'RF')
         selectFrontHind = 1;
    else selectFrontHind = 2;
end

% Set initial values for link lengths
linkLengths = initialLinkLengths;

% Set optimization options
opts = optimoptions('ga');
opts.Display = 'iter';
opts.MaxGenerations = optimizationProperties.options.maxGenerations;
opts.PopulationSize = optimizationProperties.options.populationSize;
if optimizationProperties.viz.displayBestCurrentLinkLengths
    opts.PlotFcn = {@gaplotgenealogy, @gaplotdistance, @gaplotbestf};
end
%% Set bounds and constraints
% Upper and lower angle bounds
upperBnd = round(optimizationProperties.bounds.upperBoundMultiplier.*initialLinkLengths);
lowerBnd = round(optimizationProperties.bounds.lowerBoundMultiplier.*initialLinkLengths);

%% Run optimization

costFcn = @(linkLengths)runFastJointTorqueSim(actuateJointsDirectly, l_hipAttachmentOffset, linkCount, optimizationProperties, quadruped, linkLengths, selectFrontHind, taskSelection, dt, configSelection, EEselection, meanCyclicMotionHipEE, hipParalleltoBody, Leg, meanTouchdownIndex);
disp(['Running optimization. Population: ' num2str(opts.PopulationSize) ...
      ', Max Generations: ' num2str(opts.MaxGenerations)])
%[x, feval] = ga(fun,nvars,A,b,[],[],lb,ub,nonlcon,IntCon, options)
if linkCount == 2
    nvars = [1 2 3]
elseif linkCount == 3
    nvars = [1 2 3 4]
elseif linkCount == 4
    nvars = [1 2 3 4 5];
end
 [linkLengths,penaltyMin, ~, output] = ga(costFcn,linkCount+1,[],[],[],[], ... 
                                                 lowerBnd,upperBnd,[],nvars,opts);
disp(['Final penalty function value: ' num2str(penaltyMin)])

end
