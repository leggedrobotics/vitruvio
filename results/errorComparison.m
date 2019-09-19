%% 
% RMSE = sqrt(mean((V1-V2).^2));

measData = load('ANYmalBearFastTrotMeasured.mat');
simData  = load('ANYmalBear_nom_fastTrot.mat');

timeShiftFastTrot = -2.655;
startTimeFastTrot = 1.59;
endTimeFastTrot   = 3.6;

