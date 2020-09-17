function [removalRatioStart, removalRatioEnd] = getRemovalRatios(taskSelection)
% GETREMOVALRATIOS  Returns hardcoded ratios to crop data range at the start and end of the motion. 
% The values can range from 0 to 1 but start and end values should not overlap. 

% When no other removal ratio is specified, the default is to use the
% entire motion. This value is overwritten if there exists a specified
% value.
suggestedRemovalRatioStart.(taskSelection) = 0;
suggestedRemovalRatioEnd.(taskSelection)   = 0;

%%% Add your data here (optional) %%%
suggestedRemovalRatioStart.yourTrajectoryData = 0.1;
suggestedRemovalRatioEnd.yourTrajectoryData   = 0.1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

suggestedRemovalRatioStart.ANYmalBear_fastTrot = 0.3;
suggestedRemovalRatioEnd.ANYmalBear_fastTrot   = 0.3;

suggestedRemovalRatioStart.universal_stairs = 0.3;
suggestedRemovalRatioEnd.universal_stairs   = 0;

suggestedRemovalRatioStart.massivo_stairs = 0.3;
suggestedRemovalRatioEnd.massivo_stairs   = 0.5;

suggestedRemovalRatioStart.centaur_walk = 0.2;
suggestedRemovalRatioEnd.centaur_walk   = 0.2;

suggestedRemovalRatioStart.centaur_stairs = 0.1;
suggestedRemovalRatioEnd.centaur_stairs   = 0.4;

suggestedRemovalRatioStart.mini_pronk = 0.2;
suggestedRemovalRatioEnd.mini_pronk   = 0.2;

removalRatioStart = suggestedRemovalRatioStart.(taskSelection);
removalRatioEnd = suggestedRemovalRatioEnd.(taskSelection);