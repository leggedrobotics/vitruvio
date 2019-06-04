function [linkMass, EEMass, totalLinkMass] = getLinkMass(Leg, EEselection, linkCount)
% returns mass of invidual links and the total mass of all links, excluding
% end effector and actuator mass.

numberOfBodies = linkCount + 2; % [base hip thigh shank (foot) (phalanges)]
for i = 2:numberOfBodies
    linkMass(i) = Leg.(EEselection).rigidBodyModel.Bodies{1,i}.Mass;
end
EEMass = Leg.(EEselection).rigidBodyModel.Bodies{1,end}.Mass;
totalLinkMass = sum(linkMass);

