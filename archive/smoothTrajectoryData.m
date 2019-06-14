% smooth out data from towr and save into new .mat file

% specify filepath and name of input data .mat file
inputData = load('/Users/michaelchadwick/Documents/git/vitruvio/data/motionData/ANYmalSlowTrot.mat');

EEnames = ['LF'; 'RF'; 'LH'; 'RH'];

outputData.t = inputData.t;
outputData.dt = inputData.dt;

outputData.quat = smoothdata(inputData.quat);
outputData.base.position = smoothdata(inputData.base.position);
outputData.base.velocity = smoothdata(inputData.base.velocity);
outputData.base.acceleration = smoothdata(inputData.base.acceleration);

for i = 1:4
    EEselection = EEnames(i,:);
    outputData.EE.(EEselection).position = inputData.EE.(EEselection).position;
    outputData.EE.(EEselection).velocity = inputData.EE.(EEselection).velocity;
    outputData.EE.(EEselection).force = inputData.EE.(EEselection).force; % don't need to smoothen forces
end

% save('NameOfNew.matFile', '-struct', 'outputData');