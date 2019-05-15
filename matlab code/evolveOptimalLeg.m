
function [linkLengths, penaltyMin] = evolveOptimalLeg(maxGenerations, populationSize, initialLinkLengths, upperBoundMultiplier, lowerBoundMultiplier, taskSelection, robotSelection, configSelection, EEselection, removalRatioStart, removalRatioEnd, base, quat, t, EE, dt, jointCount)
% starting quadruped properties
quadruped = getQuadrupedProperties(robotSelection);

if (EEselection == 'LF') | (EEselection == 'RF')
         selectFrontHind = 1;
    else selectFrontHind = 2;
end

%% set initial values for link lengths
linkLengths = initialLinkLengths;

opts = optimoptions('ga');
opts.Display = 'iter';
opts.MaxGenerations = maxGenerations;
opts.PopulationSize = populationSize;
% opts.InitialPopulationMatrix = repmat(p0,[5 1]); % Add copies of initial gait
% opts.PlotFcn = @gaplotbestf; % Add progress plot of fitness function
opts.PlotFcn = {@gaplotbestf, @gaplotbestindiv}; % Add progress plot of fitness function

% opts.UseParallel = parallelFlag;

%% Set bounds and constraints
% Upper and lower angle bounds
upperBnd = round(upperBoundMultiplier*initialLinkLengths);
lowerBnd = round(lowerBoundMultiplier*initialLinkLengths);

%% Run optimization
costFcn = @(linkLengths)runFastJointTorqueSim(quadruped, linkLengths, selectFrontHind, taskSelection, removalRatioStart, removalRatioEnd, base, quat, t, EE, dt, configSelection, EEselection, jointCount);
disp(['Running optimization. Population: ' num2str(opts.PopulationSize) ...
      ', Max Generations: ' num2str(opts.MaxGenerations)])
  
%[x, feval] = ga(fun,nvars,A,b,[],[],lb,ub,nonlcon,IntCon, options)
 [linkLengths,penaltyMin] = ga(costFcn,3,[],[],[],[], ... 
                              lowerBnd,upperBnd,[],[1,2,3],opts);
disp(['Final reward function value: ' num2str(penaltyMin)])


% %% Cleanup
% bdclose(mdlName);
% if parallelFlag
%    delete(gcp('nocreate')); 
% end

end
