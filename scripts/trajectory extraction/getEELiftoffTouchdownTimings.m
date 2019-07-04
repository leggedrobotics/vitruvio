% Input the raw EE force and position data. The liftoff and touchdown
% timings for each step of each leg are computed and returned along with
% the minimum number of steps taken by any leg.
function [tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(Leg, EE, EEnames, legCount)
    t = Leg.time;
    minStepCount = [];
    
    for i = 1:legCount
        EEselection = EEnames(i,:);
        tLiftoff.(EEselection) = 0; % in the case where there are no liftoffs such as for a pushup motion
        j = 1; % liftoff index
        k = 1; % touchdown index
      
        %% liftoff
        % if force at time t is non zero and force at t+1 is zero, the foot
        % lifted off 
        for i=2:length(EE.(EEselection).force(:,3))
            if(EE.(EEselection).force(i-1,3) ~=0 ) && (EE.(EEselection).force(i,3) == 0)
                tLiftoff.(EEselection)(j) = t(i);
                j = j+1;
            end
        %% touchdown    
        % if force at time t is zero and force at t+1 is non zero, the foot
        % touched down 
            if(EE.(EEselection).force(i-1,3) ==0 ) && (EE.(EEselection).force(i,3) ~= 0)
                tTouchdown.(EEselection)(k) = t(i);
                k = k+1;
            end
        end
        minStepCountTemp.(EEselection) = length(tLiftoff.(EEselection));
        minStepCount = [minStepCount, minStepCountTemp.(EEselection)];
    end
    % the minimum number of steps of any leg
    minStepCount = min(minStepCount);

    %% the liftoff and touchdown timings are only returned from steps 1 to the minimum number of steps
    for i = 1:legCount
        tLiftoff.(EEselection) = tLiftoff.(EEselection)(1:minStepCount)';
        tTouchdown.(EEselection) = tTouchdown.(EEselection)(1:minStepCount)';
    end
end