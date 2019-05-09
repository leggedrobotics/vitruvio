function  q0 = getInitialJointAnglesForDesiredConfig(taskSelection, EEselection, configSelection)
% these work for the nominal lenghts but may be incorrect when link lengths
% change
q0.X.universalStairs.LF = [0 -pi/4 -pi/2 0];
q0.X.universalStairs.LH = [0 -pi/4 -pi/2 0];
q0.X.universalStairs.RF = [0 -pi/4 -pi/2 0];
q0.X.universalStairs.RH = [0 -pi/4 -pi 0];

q0.X.universalTrot.LF = [0 -pi/4 -pi/2 0];
q0.X.universalTrot.LH = [0 -pi/4 -pi 0];
q0.X.universalTrot.RF = [0 -pi/4 -pi/2 0];
q0.X.universalTrot.RH = [0 -pi/4 -pi 0];

q0.M.universalTrot.LF = [0 -pi/4 -pi/2 0];
q0.M.universalTrot.LH = [0 -pi/4 -pi/2 0];
q0.M.universalTrot.RF = [0 -pi/4 -pi/2 0];
q0.M.universalTrot.RH = [0 -pi/4 -pi/2 0];

q0 = q0.(configSelection).(taskSelection).(EEselection);