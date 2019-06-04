function qAFE = computeAFE(Leg, EEselection, configSelection)

qAFE_max = pi/4;
F_max = max(Leg.(EEselection).force(:,3));
r = Leg.(EEselection).force(:,3)/F_max; % z direction forces
qAFE = r * qAFE_max;

% add pi/2 to angle for hind legs in X configuration. This ensures that the
% feet are pointing forward.
if (EEselection == 'LF') | (EEselection == 'RF')
     selectFrontHind = 1;
    else
     selectFrontHind = 2;
end

if configSelection == 'X' && selectFrontHind == 2
    qAFE = qAFE + pi/3;
end
end

