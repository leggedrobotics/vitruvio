function [removalRatioStart, removalRatioEnd] = getRemovalRatios(taskSelection)
% GETREMOVALRATIOS  Returns hardcoded ratios to crop data range at the start and end of the motion. The values can range from 0 to 1 but start and end values should not overlap. 

% When no other removal ratio is specified, the default is to use the
% entire motion. This value is overwritten if there exists a specified
% value.
suggestedRemovalRatioStart.(taskSelection) = 0;
suggestedRemovalRatioEnd.(taskSelection) = 0;

%%% Add your data here (optional) %%%
suggestedRemovalRatioStart.yourTrajectoryData = 0.1;
suggestedRemovalRatioEnd.yourTrajectoryData   = 0.1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

suggestedRemovalRatioStart.universal_trot = 0.2;
suggestedRemovalRatioEnd.universal_trot   = 0.2;

suggestedRemovalRatioStart.universal_stairs = 0.3;
suggestedRemovalRatioEnd.universal_stairs   = 0*0.3;

suggestedRemovalRatioStart.speedy_gallop = 0.3;
suggestedRemovalRatioEnd.speedy_gallop   = 0.5;

suggestedRemovalRatioStart.speedy_stairs = 0.2;
suggestedRemovalRatioEnd.speedy_stairs   = 0.2;

suggestedRemovalRatioStart.massivo_walk = 0.2;
suggestedRemovalRatioEnd.massivo_walk   = 0.2;

suggestedRemovalRatioStart.massivo_stairs = 0.2;
suggestedRemovalRatioEnd.massivo_stairs   = 0.2;

suggestedRemovalRatioStart.centaur_stairs = 0.2;
suggestedRemovalRatioEnd.centaur_stairs   = 0.2;

suggestedRemovalRatioStart.centaur_walk = 0.2;
suggestedRemovalRatioEnd.centaur_walk   = 0.2;

suggestedRemovalRatioStart.mini_pronk = 0.1;
suggestedRemovalRatioEnd.mini_pronk   = 0.6;

suggestedRemovalRatioStart.ANYmalBear_trot = 0.3;
suggestedRemovalRatioEnd.ANYmalBear_trot   = 0.3;

suggestedRemovalRatioStart.ANYmalBearFast_trot = 0.2; %0.3;
suggestedRemovalRatioEnd.ANYmalBearFast_trot   = 0; %0.3;

suggestedRemovalRatioStart.ANYmalBear_fastTrotExtendedxNom3 = 0; %0.3; %0.3;
suggestedRemovalRatioEnd.ANYmalBear_fastTrotExtendedxNom3   = 0; %0.2; %0.3;

suggestedRemovalRatioStart.ANYmalBear_slowTrot = 0; %0.3;
suggestedRemovalRatioEnd.ANYmalBear_slowTrot   = 0; %0.3;

suggestedRemovalRatioStart.vitruvianBiped_pushupSquat = 0;
suggestedRemovalRatioEnd.vitruvianBiped_pushupSquat   = 0.05;

removalRatioStart = suggestedRemovalRatioStart.(taskSelection);
removalRatioEnd = suggestedRemovalRatioEnd.(taskSelection);