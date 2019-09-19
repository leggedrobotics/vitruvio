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
suggestedRemovalRatioEnd.universalTrot   = 0;

suggestedRemovalRatioStart.universalStairs = 0;%0.3;
suggestedRemovalRatioEnd.universalStairs   = 0;%0.3;

suggestedRemovalRatioStart.speedyGallop = 0.3;
suggestedRemovalRatioEnd.speedyGallop   = 0.5;

suggestedRemovalRatioStart.speedyStairs = 0.2;
suggestedRemovalRatioEnd.speedyStairs   = 0.2;

suggestedRemovalRatioStart.massivoWalk = 0.2;
suggestedRemovalRatioEnd.massivoWalk   = 0;

suggestedRemovalRatioStart.massivoStairs = 0.1;
suggestedRemovalRatioEnd.massivoStairs   = 0;

suggestedRemovalRatioStart.centaurStairs = 0.1;
suggestedRemovalRatioEnd.centaurStairs   = 0;

suggestedRemovalRatioStart.centaurWalk = 0.2;
suggestedRemovalRatioEnd.centaurWalk   = 0;

suggestedRemovalRatioStart.miniPronk = 0.1;
suggestedRemovalRatioEnd.miniPronk   = 0;

suggestedRemovalRatioStart.ANYmalTrot = 0.1;
suggestedRemovalRatioEnd.ANYmalTrot   = 0;

suggestedRemovalRatioStart.ANYmalSlowTrotAccurateMotion = 0;
suggestedRemovalRatioEnd.ANYmalSlowTrotAccurateMotion   = 0;

suggestedRemovalRatioStart.ANYmalSlowTrot2 = 0.3;
suggestedRemovalRatioEnd.ANYmalSlowTrot2   = 0.3;

suggestedRemovalRatioStart.ANYmalFlyingTrot = 0.4;
suggestedRemovalRatioEnd.ANYmalFlyingTrot   = 0.4;

suggestedRemovalRatioStart.ANYmalTrotVersatilityStep = 0.6;
suggestedRemovalRatioEnd.ANYmalTrotVersatilityStep   = 0.1;

suggestedRemovalRatioStart.ANYmalBearTrot = 0.3;
suggestedRemovalRatioEnd.ANYmalBearTrot   = 0.3;

suggestedRemovalRatioStart.ANYmalBearFastTrot = 0.2; %0.3;
suggestedRemovalRatioEnd.ANYmalBearFastTrot   = 0; %0.3;

suggestedRemovalRatioStart.ANYmalBearFastTrotExtendedxNom = 0.2; %0.3;
suggestedRemovalRatioEnd.ANYmalBearFastTrotExtendedxNom   = 0; %0.3;

suggestedRemovalRatioStart.ANYmalBearFastTrotExtendedxNom3 = 0.3;
suggestedRemovalRatioEnd.ANYmalBearFastTrotExtendedxNom3   = 0.3;

suggestedRemovalRatioStart.ANYmalBearSlowTrot = 0.2; %0.3;
suggestedRemovalRatioEnd.ANYmalBearSlowTrot   = 0.2; %0.3;

suggestedRemovalRatioStart.ANYmalBearSlowTrotIntermediateTorque = 0.3;
suggestedRemovalRatioEnd.ANYmalBearSlowTrotIntermediateTorque   = 0.3;

suggestedRemovalRatioStart.ANYmalBearElongatedTrot = 0.1;
suggestedRemovalRatioEnd.ANYmalBearElongatedTrot = 0.1;

suggestedRemovalRatioStart.ANYmalBearFlyingTrot2 = 0.3;
suggestedRemovalRatioEnd.ANYmalBearFlyingTrot2  = 0.3;


suggestedRemovalRatioStart.vitruvianBipedPushupSquat = 0;
suggestedRemovalRatioEnd.vitruvianBipedPushupSquat   = 0.05;

suggestedRemovalRatioStart.vitruvianBipedWalkOnSpot = 0;
suggestedRemovalRatioEnd.vitruvianBipedWalkOnSpot   = 0;

suggestedRemovalRatioStart.vitruvianBipedHop = 0;
suggestedRemovalRatioEnd.vitruvianBipedHop   = 0;

removalRatioStart = suggestedRemovalRatioStart.(taskSelection);
removalRatioEnd = suggestedRemovalRatioEnd.(taskSelection);