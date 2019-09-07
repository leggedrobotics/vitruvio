function reachablePositions = getRangeofMotion(data)
    EEnames   = data.basicProperties.EEnames;
    legCount  = data.basicProperties.legCount;
    linkCount = data.basicProperties.linkCount;
    stepSize = 0.2;
    %% Initialize using position of HFE at first time step.
    for i = 1:legCount
        EEselection = EEnames(i,:);
        HFExPos.(EEselection) = data.(EEselection).r.HFE(1,1); % initial x position of HFE joint
        HFEyPos.(EEselection) = data.(EEselection).r.HFE(1,2); % initial y position of HFE joint
        HFEzPos.(EEselection) = data.(EEselection).r.HFE(1,3); % initial z position of HFE joint
        
        %% Read in leg link lengths
        l_hip.(EEselection)       = data.(EEselection).linkLengths(1);
        l_thigh.(EEselection)     = data.(EEselection).linkLengths(2);
        l_shank.(EEselection)     = data.(EEselection).linkLengths(3);
        if linkCount > 2
            l_foot.(EEselection)  = data.(EEselection).linkLengths(4);
        end
        if linkCount > 3
            l_phalanges.(EEselection) = data.(EEselection).linkLengths(5);
        end

        %% Read in max and min joint angles from quadruped properties
        q2_min = data.(EEselection).qLimits.min(2);
        q2_max = data.(EEselection).qLimits.max(2);
        q3_min = data.(EEselection).qLimits.min(3);
        q3_max = data.(EEselection).qLimits.max(3);

        q2 = q2_min:stepSize:q2_max; % all possible HFE
        q3 = q3_min:stepSize:q3_max; % all possible KFE

        if linkCount > 2
            q4_min = data.(EEselection).qLimits.min(4);
            q4_max = data.(EEselection).qLimits.max(4);
            q4 = q4_min:stepSize:q4_max; % all possible AFE
        end

        if linkCount > 3
            q5_min = data.(EEselection).qLimits.min(5);
            q5_max = data.(EEselection).qLimits.max(5);
            q5 = q5_min:stepSize:q5_max; % all possible DFE
        end

        %% Create a grid of the joint angles
        if linkCount == 2
            [q2_, q3_] = ndgrid(q2, q3);
        elseif linkCount == 3
            [q2_, q3_, q4_] = ndgrid(q2, q3, q4);
        else
            [q2_, q3_, q4_, q5_] = ndgrid(q2, q3, q4, q5);
        end
        
        %% Compute the possible EE positions for all joint angle combinations
        % this neglects HAA angle
        X.(EEselection) = HFExPos.(EEselection) + l_thigh.(EEselection) * sin(q2_) + l_shank.(EEselection) * sin(q2_ + q3_); 
        Z.(EEselection) = HFEzPos.(EEselection) - l_thigh.(EEselection) * cos(q2_) - l_shank.(EEselection) * cos(q2_ + q3_);

        if linkCount > 2
            X.(EEselection) = X.(EEselection) + l_foot.(EEselection) * sin(q2_ + q3_ + q4_);
            Z.(EEselection) = Z.(EEselection) - l_foot.(EEselection) * cos(q2_ + q3_ + q4_);
        end
        if linkCount > 3
            X.(EEselection) = X.(EEselection) + l_phalanges.(EEselection) * sin(q2_ + q3_ + q4_ + q5_);
            Z.(EEselection) = Z.(EEselection) - l_phalanges.(EEselection) * cos(q2_ + q3_ + q4_ + q5_);
        end
        % save reachable positions for front and hind legs
        reachablePositions.(EEselection) = [X.(EEselection)(:) Z.(EEselection)(:)];
    end
end
