function [legDesignParameters, penaltyMin, output] = evolveOptimalLeg(actuatorProperties, imposeJointLimits, heuristic, upperBound, lowerBound, actuateJointDirectly, linkCount, optimizationProperties, initialLinkLengths, taskSelection, robotProperties, configSelection, EEselection, dt, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency , actuatorSelection, dataExtraction, jointNames, springInParallelWithJoints, kSpringJoint, q0SpringJoint)
    if strcmp(EEselection, 'LF') || strcmp(EEselection, 'RF')
        selectFrontHind = 1;
    else 
        selectFrontHind = 2;
    end
    transmissionGearRatio = robotProperties.transmissionGearRatio.HAA(selectFrontHind);
    for i = 2:linkCount+1
        transmissionGearRatio = [transmissionGearRatio, robotProperties.transmissionGearRatio.(jointNames(i,:))(selectFrontHind)];
    end

    % Set initial values for link lengths
    linkLengths = initialLinkLengths;
    legDesignParameters = [linkLengths, transmissionGearRatio];

    % If also optimizing joint spring parameters
    if springInParallelWithJoints
        legDesignParameters = [legDesignParameters kSpringJoint.(EEselection)(1:linkCount+1)];
    end

    % If also optimizing spring parameters at AFE/DFE
    if linkCount>2 && heuristic.torqueAngle.apply
        kTorsionalSpring = heuristic.torqueAngle.kTorsionalSpring;
        thetaLiftoff_des = heuristic.torqueAngle.thetaLiftoff_des;
        legDesignParameters = [legDesignParameters, kTorsionalSpring, thetaLiftoff_des];
    end

    % Set optimization options
    % parpool('local')
    opts = optimoptions('ga', 'UseParallel', true, 'UseVectorized', false);
    opts.Display = 'iter';
    opts.MaxGenerations = optimizationProperties.options.maxGenerations;
    opts.PopulationSize = optimizationProperties.options.populationSize;
    opts.CreationFcn  = {@gacreationuniform};
    opts.MutationFcn  = {@mutationadaptfeasible};

    if optimizationProperties.viz.displayBestCurrentDesign
        opts.PlotFcn = {@gaplotbestindiv};
    end

    %% Run optimization
    % Compute penalty runs the simulation using legDesignParameters to obtain 
    % joint data and evaluates the penalty function for the simulated result. 
    costFcn = @(legDesignParameters)computePenalty(actuatorProperties, imposeJointLimits, heuristic, legDesignParameters, actuateJointDirectly, linkCount, optimizationProperties, robotProperties, selectFrontHind, taskSelection, dt, configSelection, EEselection, meanCyclicMotionHipEE, hipParalleltoBody, Leg, actuatorEfficiency, actuatorSelection, dataExtraction, springInParallelWithJoints, kSpringJoint, q0SpringJoint);
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