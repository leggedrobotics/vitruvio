function  q0 = getInitialJointAnglesForDesiredConfig(taskSelection, EEselection, configSelection)
% these work for the nominal lengths but may be incorrect when link lengths
% change

%% universal

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

%% speedy
q0.M.speedyGallop.LF = [0 -pi/4 -pi/2 0];
q0.M.speedyGallop.LH = [0 pi/4  -pi/2 0];
q0.M.speedyGallop.RF = [0 -pi/4 -pi/2 0];
q0.M.speedyGallop.RH = [0 pi/4  -pi/2 0];

q0.X.speedyGallop.LF = [0 -pi/4 -pi/2 0];
q0.X.speedyGallop.LH = [0 -pi/4 -pi 0];
q0.X.speedyGallop.RF = [0 -pi/4 -pi/2 0];
q0.X.speedyGallop.RH = [0 -pi/4 -pi 0];

q0.X.speedyStairs = q0.X.speedyGallop;
q0.M.speedyStairs = q0.M.speedyGallop;

%% massivo
q0.X.massivoWalk.LF = [0 -pi/4 -pi/2 0];
q0.X.massivoWalk.LH = [0 pi/4  -pi/2 0];
q0.X.massivoWalk.RF = [0 -pi/4 -pi/2 0];
q0.X.massivoWalk.RH = [0 pi/4  -pi/2 0];

q0.M.massivoWalk.LF = [0 -pi/4 -pi/2 0];
q0.M.massivoWalk.LH = [0 pi/4  -pi/2 0];
q0.M.massivoWalk.RF = [0 -pi/4 -pi/2 0];
q0.M.massivoWalk.RH = [0 pi/4  -pi/2 0];

q0.X.massivoStairs.LF = [0 -pi/4 -pi/2 0];
q0.X.massivoStairs.LH = [0 -pi/4 -pi 0];
q0.X.massivoStairs.RF = [0 -pi/4 -pi/2 0];
q0.X.massivoStairs.RH = [0 -pi/4 -pi 0];

q0 = q0.(configSelection).(taskSelection).(EEselection);