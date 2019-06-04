function  q0 = getInitialJointAnglesForDesiredConfig(taskSelection, EEselection, configSelection)

% these work for the nominal lengths but may be incorrect when link lengths
% the values are mostly just placeholders and need to be updated

kneesForward = [0  0.64 -4.4  0];
kneesBackward = [0  -3.8 -0.66 0];

q0.X.quadruped.LF = kneesBackward;
q0.X.quadruped.LH = kneesForward;
q0.X.quadruped.RF = kneesBackward;
q0.X.quadruped.RH = kneesForward;

q0.M.quadruped.LF = kneesBackward;
q0.M.quadruped.LH = kneesBackward;
q0.M.quadruped.RF = kneesBackward;
q0.M.quadruped.RH = kneesBackward;

%% X configuration
q0.X.universalStairs    = q0.X.quadruped;
q0.X.universalTrot      = q0.X.quadruped;
q0.X.speedyGallop       = q0.X.quadruped;
q0.X.speedyStairs       = q0.X.quadruped;
q0.X.massivoWalk        = q0.X.quadruped;
q0.X.massivoStairs      = q0.X.quadruped;
q0.X.centaurWalk        = q0.X.quadruped;
q0.X.centaurStairs      = q0.X.quadruped;
q0.X.miniPronk          = q0.X.quadruped;

%% M configuration
q0.M.universalStairs    = q0.M.quadruped;
q0.M.universalTrot      = q0.M.quadruped;
q0.M.speedyGallop       = q0.M.quadruped;
q0.M.speedyStairs       = q0.M.quadruped;
q0.M.massivoWalk        = q0.M.quadruped;
q0.M.massivoStairs      = q0.M.quadruped;
q0.M.centaurWalk        = q0.M.quadruped;
q0.M.centaurStairs      = q0.M.quadruped;
q0.M.miniPronk          = q0.M.quadruped;

q0 = q0.(configSelection).(taskSelection).(EEselection);