% Save optimized results for plotting against nominal
robotClass = 'ANYmalBear';
robotTask  = 'trotSwing5';

for i = 1:legCount
    EEselection = EEnames(i,:);
    resultsOpt.(EEselection).jointTorqueOpt = results.(robotClass).(robotTask).(EEselection).actuatorTorque;
    resultsOpt.(EEselection).mechEnergyOpt = results.(robotClass).(robotTask).(EEselection).mechEnergy;
end