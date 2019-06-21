function [removalRatioStart, removalRatioEnd] = getSuggestedRemovalRatios(taskSelection)
% GETSUGGESTEDREMOVALRATIOS  Returns hardcoded ratios to crop data range.
%   [removalRatioStart, removalRatioEnd] = GETSUGGESTEDREMOVALRATIOS(taskSelection) returns removal ratios for selected task.

suggestedRemovalRatioStart.universalStairs = 0.1;
suggestedRemovalRatioEnd.universalStairs = 0.1;

suggestedRemovalRatioStart.universalTrot = 0.4;
suggestedRemovalRatioEnd.universalTrot = 0.1;

suggestedRemovalRatioStart.speedyGallop = 0.3;
suggestedRemovalRatioEnd.speedyGallop = 0.1;

suggestedRemovalRatioStart.speedyStairs = 0.5;
suggestedRemovalRatioEnd.speedyStairs = 0.2;

suggestedRemovalRatioStart.massivoWalk = 0.2;
suggestedRemovalRatioEnd.massivoWalk = 0.1;

suggestedRemovalRatioStart.massivoStairs = 0.3;
suggestedRemovalRatioEnd.massivoStairs = 0.6;

suggestedRemovalRatioStart.centaurStairs = 0.3;
suggestedRemovalRatioEnd.centaurStairs = 0.6;

suggestedRemovalRatioStart.centaurWalk = 0.2;
suggestedRemovalRatioEnd.centaurWalk = 0.1;

suggestedRemovalRatioStart.miniPronk = 0.1;
suggestedRemovalRatioEnd.miniPronk = 0.1;

suggestedRemovalRatioStart.ANYmalTrot = 0.4;
suggestedRemovalRatioEnd.ANYmalTrot = 0.5;

suggestedRemovalRatioStart.ANYmalSlowTrot = 0.2;
suggestedRemovalRatioEnd.ANYmalSlowTrot = 0.2;

suggestedRemovalRatioStart.ANYmalSlowTrotGoodMotionBadForce = 0.2;
suggestedRemovalRatioEnd.ANYmalSlowTrotGoodMotionBadForce = 0.2;

suggestedRemovalRatioStart.ANYmalSlowTrotOriginal = 0.2;
suggestedRemovalRatioEnd.ANYmalSlowTrotOriginal = 0.2;

removalRatioStart = suggestedRemovalRatioStart.(taskSelection);
removalRatioEnd = suggestedRemovalRatioEnd.(taskSelection);