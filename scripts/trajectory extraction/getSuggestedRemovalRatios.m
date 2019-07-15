function [removalRatioStart, removalRatioEnd] = getSuggestedRemovalRatios(taskSelection)
% GETSUGGESTEDREMOVALRATIOS  Returns hardcoded ratios to crop data range at the start and end of the motion. The values can range from 0 to 1 but start and end values should not overlap. 
%   [removalRatioStart, removalRatioEnd] = GETSUGGESTEDREMOVALRATIOS(taskSelection) returns removal ratios for selected task.

% When no other removal ratio is specified, the default is to use the
% entire motion. This value is overwritten if there exists a specified
% value.
suggestedRemovalRatioStart.(taskSelection) = 0;
suggestedRemovalRatioEnd.(taskSelection) = 0;

%%% Add your data here (optional) %%%
suggestedRemovalRatioStart.yourTrajectoryData = 0.1;
suggestedRemovalRatioEnd.yourTrajectoryData   = 0.1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

suggestedRemovalRatioStart.universalTrot = 0.1;
suggestedRemovalRatioEnd.universalTrot   = 0.1;

suggestedRemovalRatioStart.universalStairs = 0.4;
suggestedRemovalRatioEnd.universalStairs   = 0.1;

suggestedRemovalRatioStart.speedyGallop = 0.3;
suggestedRemovalRatioEnd.speedyGallop   = 0.1;

suggestedRemovalRatioStart.speedyStairs = 0.2;
suggestedRemovalRatioEnd.speedyStairs   = 0.2;

suggestedRemovalRatioStart.massivoWalk = 0.2;
suggestedRemovalRatioEnd.massivoWalk   = 0.1;

suggestedRemovalRatioStart.massivoStairs = 0.1;
suggestedRemovalRatioEnd.massivoStairs   = 0.6;

suggestedRemovalRatioStart.centaurStairs = 0.1;
suggestedRemovalRatioEnd.centaurStairs   = 0.6;

suggestedRemovalRatioStart.centaurWalk = 0.2;
suggestedRemovalRatioEnd.centaurWalk   = 0.1;

suggestedRemovalRatioStart.miniPronk = 0.1;
suggestedRemovalRatioEnd.miniPronk   = 0.1;

suggestedRemovalRatioStart.ANYmalTrot = 0.4;
suggestedRemovalRatioEnd.ANYmalTrot   = 0.5;

suggestedRemovalRatioStart.ANYmalSlowTrotAccurateMotion = 0;
suggestedRemovalRatioEnd.ANYmalSlowTrotAccurateMotion   = 0.1;

suggestedRemovalRatioStart.ANYmalSlowTrot2 = 0.3;
suggestedRemovalRatioEnd.ANYmalSlowTrot2   = 0.3;

suggestedRemovalRatioStart.defaultHopperHop = 0;
suggestedRemovalRatioEnd.defaultHopperHop   = 0;

removalRatioStart = suggestedRemovalRatioStart.(taskSelection);
removalRatioEnd = suggestedRemovalRatioEnd.(taskSelection);