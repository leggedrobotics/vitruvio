function  q0 = getInitialJointAnglesForDesiredConfig(EEselection, configSelection)

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

q0 = q0.(configSelection).quadruped.(EEselection);