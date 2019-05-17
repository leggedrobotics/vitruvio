
function [linkLengths, penaltyMin] = evolveOptimalLeg(maxGenerations, populationSize, initialLinkLengths, upperBoundMultiplier, lowerBoundMultiplier, taskSelection, quadruped, configSelection, EEselection, EE, dt, jointCount, meanCyclicMotionHipEE)

if (EEselection == 'LF') | (EEselection == 'RF')
         selectFrontHind = 1;
    else selectFrontHind = 2;
end

% Set initial values for link lengths
linkLengths = initialLinkLengths;

% Set optimization options
opts = optimoptions('ga');
opts.Display = 'iter';
opts.MaxGenerations = maxGenerations;
opts.PopulationSize = populationSize;
opts.PlotFcn = {@gaplotbestf, @gaplotbestindiv};

%% Set bounds and constraints
% Upper and lower angle bounds
upperBnd = round(upperBoundMultiplier*initialLinkLengths);
lowerBnd = round(lowerBoundMultiplier*initialLinkLengths);

%% Run optimization
costFcn = @(linkLengths)runFastJointTorqueSim(quadruped, linkLengths, selectFrontHind, taskSelection, EE, dt, configSelection, EEselection, jointCount, meanCyclicMotionHipEE);
disp(['Running optimization. Population: ' num2str(opts.PopulationSize) ...
      ', Max Generations: ' num2str(opts.MaxGenerations)])
  
%[x, feval] = ga(fun,nvars,A,b,[],[],lb,ub,nonlcon,IntCon, options)
 [linkLengths,penaltyMin] = ga(costFcn,3,[],[],[],[], ... 
                              lowerBnd,upperBnd,[],[1,2,3],opts);
disp(['Final reward function value: ' num2str(penaltyMin)])

end
