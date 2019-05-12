taskSelection = 'massivoWalk';
robotSelection = 'massivo';
[removalRatioStart, removalRatioEnd] = getSuggestedRemovalRatios(taskSelection);
load(taskSelection);
dt = t(2) - t(1);
configSelection = 'X';
EEselection = 'LF';
EEcount = 1;


jointTorque = runFastJointTorqueSim(taskSelection, removalRatioStart, removalRatioEnd, base, quat, robotSelection, t, EE, dt, configSelection, EEselection, EEcount);
