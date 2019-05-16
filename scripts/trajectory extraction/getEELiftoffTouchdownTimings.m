%% getEELiftoffTouchdownTimings
function [tLiftoff, tTouchdown, minStepCount] = getEELiftoffTouchdownTimings(t, EE)

%% LF
j = 1;
k = 1;
% if force at time t is non zero and force at t+1 is zero, the foot
% lifted off 
for i=2:length(EE.LF.force(:,3))
    if(EE.LF.force(i-1,3) ~=0 ) && (EE.LF.force(i,3) == 0)
        tLiftoffLF(j) = t(i);
        j = j+1;
    end
    
    if(EE.LF.force(i-1,3) ==0 ) && (EE.LF.force(i,3) ~= 0)
        tTouchdownLF(k) = t(i);
        k = k+1;
    end
end


%% LH
j = 1;
k = 1;
for i=2:length(EE.LH.force(:,3))
    if(EE.LH.force(i-1,3) ~=0 ) && (EE.LH.force(i,3) == 0)
        tLiftoffLH(j) = t(i);
        j = j+1;
    end
    
    if(EE.LH.force(i-1,3) ==0 ) && (EE.LH.force(i,3) ~= 0)
        tTouchdownLH(k) = t(i);
        k = k+1;
    end
end

%% RF
j = 1;
k = 1;
for i=2:length(EE.RF.force(:,3))
    if(EE.RF.force(i-1,3) ~=0 ) && (EE.RF.force(i,3) == 0)
        tLiftoffRF(j) = t(i);
        j = j+1;
    end
    
    if(EE.RF.force(i-1,3) ==0 ) && (EE.RF.force(i,3) ~= 0)
        tTouchdownRF(k) = t(i);
        k = k+1;
    end
end

%% RH
j = 1;
k = 1;
for i=2:length(EE.RH.force(:,3))
    if(EE.RH.force(i-1,3) ~=0 ) && (EE.RH.force(i,3) == 0)
        tLiftoffRH(j) = t(i);
        j = j+1;
    end
    
    if(EE.RH.force(i-1,3) ==0 ) && (EE.RH.force(i,3) ~= 0)
        tTouchdownRH(k) = t(i);
        k = k+1;
    end
end

% sometimes some legs have fewer steps than others. Get rid of additional
% step data from other legs so that each leg has the same number of
% liftoffs and touchdowns
minStepCount = min([length(tLiftoffLF), length(tLiftoffLH), length(tLiftoffRF),length( tLiftoffRH)]);

% row order:    LF LH RF RH
tLiftoff = [tLiftoffLF(1:minStepCount)' tLiftoffLH(1:minStepCount)' tLiftoffRF(1:minStepCount)' tLiftoffRH(1:minStepCount)'];
tTouchdown = [tTouchdownLF(1:minStepCount)' tTouchdownLH(1:minStepCount)' tTouchdownRF(1:minStepCount)' tTouchdownRH(1:minStepCount)'];