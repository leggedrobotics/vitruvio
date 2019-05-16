function  q0 = getInitialJointAnglesForDesiredConfig(taskSelection, EEselection, configSelection)

% these work for the nominal lengths but may be incorrect when link lengths
% the values are mostly just placeholders and need to be updated

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

q0.M.universalStairs = q0.M.universalTrot;

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

q0.X.massivoStairs = q0.X.massivoWalk;
q0.M.massivoStairs = q0.M.massivoWalk;

%% centaur
q0.X.centaurWalk = q0.X.massivoWalk;
q0.M.centaurWalk = q0.M.massivoWalk;
q0.X.centaurStairs = q0.X.massivoStairs;
q0.M.centaurStairs = q0.M.massivoStairs;

%% mini
q0.X.miniPronk = q0.X.massivoWalk;
q0.M.miniPronk = q0.M.massivoWalk;


q0 = q0.(configSelection).(taskSelection).(EEselection);