% Input the raw EE force and position data. The liftoff and touchdown
% timings for each step of each leg are computed and returned along with
% the minimum number of steps taken by any leg.
function [tLiftoff, tTouchdown, minStepCountTemp] = getEELiftoffTouchdownTimings(t, force)
    tol = 0.0001; % [Nm] Forces below tol are considered to be zero
    
    tLiftoff = 0; % in the case where there are no liftoffs such as for a pushup motion we need to initialize the values
    tTouchdown = 0;
    j = 1; % liftoff index
    k = 1; % touchdown index
      
    %% liftoff
    % if force at time t is non zero and force at t+1 is zero, the foot
    % lifted off 
    for i=2:length(force(:,3))
        if(force(i-1,3) > 0 ) && (force(i,3) == 0)
            tLiftoff(j) = t(i);
            j = j+1;
        end

        %% touchdown
    % if force at time t is zero and force at t+1 is non zero, the foot
    % touched down 
        if(force(i-1,3) == 0 ) && (force(i,3) > 0)
            tTouchdown(k) = t(i);
            k = k+1;
        end
    end
    minStepCountTemp = length(tLiftoff);
end