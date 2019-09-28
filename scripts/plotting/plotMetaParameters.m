function [] = plotMetaParameters(data, saveFiguresToPDF)
    t                    = data.time;
    base                 = data.base; % base motion for each leg during its cycle
    base.fullTrajectory  = data.fullTrajectory.base;
    force.fullTrajectory = data.fullTrajectory.force;
    legCount             = data.basicProperties.legCount;
    linkCount            = data.basicProperties.linkCount;
    EEnames              = data.basicProperties.EEnames;
    removalRatioStart    = data.basicProperties.trajectory.removalRatioStart;
    removalRatioEnd      = data.basicProperties.trajectory.removalRatioEnd;
    fontSize = 16; 

    if saveFiguresToPDF
        outerPosition = [0 0 1 1]; % Fullscreen
    else
        outerPosition = [0.5 0.5 0.5 0.5]; % Top right corner
    end
    
    %% Link Lengths
    if linkCount == 2
        linkLabels = {'Hip','Thigh','Shank'};
        jointLabels = {'HAA', 'HFE', 'KFE'};
    elseif linkCount == 3
        linkLabels = {'Hip','Thigh','Shank', 'Foot'};
        jointLabels = {'HAA', 'HFE', 'KFE', 'AFE'};
    elseif linkCount == 4
        linkLabels = {'Hip','Thigh','Shank', 'Foot'};
        jointLabels = {'HAA', 'HFE', 'KFE', 'AFE', 'DFE'};
    end
    categoryLabels = categorical({'Nominal', 'Optimized'});

    figureName = 'Link Length Comparison';
    figure('name', figureName, 'DefaultAxesFontSize', 10, 'units','normalized','outerposition',outerPosition)
    set(gcf,'color','w')
    for i = 1:legCount
        EEselection = EEnames(i,:);
        if data.basicProperties.optimizedLegs.(EEselection)
            linkLengths.(EEselection) = [100*data.(EEselection).linkLengths; 100*data.(EEselection).linkLengthsOpt];
            maxTorque.(EEselection)   = [data.metaParameters.jointTorqueMax.(EEselection);  data.metaParameters.jointTorqueMaxOpt.(EEselection)];
            mechEnergy.(EEselection)  = [data.metaParameters.mechEnergyPerCycle.(EEselection); data.metaParameters.mechEnergyPerCycleOpt.(EEselection)];
            elecEnergy.(EEselection)  = [data.metaParameters.elecEnergyPerCycle.(EEselection); data.metaParameters.elecEnergyPerCycleOpt.(EEselection)];
        else
            linkLengths.(EEselection) = [100*data.(EEselection).linkLengths; zeros(1, length(data.(EEselection).linkLengths))];
            maxTorque.(EEselection)   = [data.metaParameters.jointTorqueMax.(EEselection); zeros(1, length(data.(EEselection).linkLengths))];
            mechEnergy.(EEselection)  = [data.metaParameters.mechEnergyPerCycle.(EEselection); zeros(1, length(data.(EEselection).linkLengths))];
            elecEnergy.(EEselection)  = [data.metaParameters.elecEnergyPerCycle.(EEselection); zeros(1, length(data.(EEselection).linkLengths))];
        end

        subplot(2,2,i)
        bar(categoryLabels, linkLengths.(EEselection), 'stacked');
        legend(linkLabels,'Location','southoutside','Orientation','horizontal')
        title((['Link Length Comparison ', EEselection]), 'FontSize', fontSize);
        ylabel('Link Lengths [cm]');
    end
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end
        
    %% Peak joint torque comparison of actuators
    figureName = 'Joint Torque Comparison';
    figure('name', figureName, 'DefaultAxesFontSize', 10, 'units','normalized','outerposition', outerPosition)
    set(gcf,'color','w')

    for i = 1:legCount
        EEselection = EEnames(i,:);
        subplot(2,2,i)
        bar(categoryLabels, maxTorque.(EEselection), 'stacked');
        legend(jointLabels,'Location','southoutside','Orientation','horizontal')
        title((['Joint Torque Comparison ', EEselection]), 'FontSize', fontSize);
        ylabel('Peak Joint Torque [Nm]');
    end
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end

    %% Energy consumption comparison of actuators
    % Mechanical energy demand        
    figureName = 'Mechanical Energy Comparison';
    figure('name', figureName, 'DefaultAxesFontSize', 10, 'units','normalized','outerposition', outerPosition)
    set(gcf,'color','w')

    for i = 1:legCount
        EEselection = EEnames(i,:);
        subplot(2,2,i)
        bar(categoryLabels, mechEnergy.(EEselection), 'stacked');
        legend(jointLabels,'Location','southoutside','Orientation','horizontal')
        title((['Mechanical Energy Comparison ', EEselection]), 'FontSize', fontSize);
        ylabel('Energy [J/cycle]');
    end
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end

    % Electrical energy demand
    figureName = 'Electrical Energy Comparison';
    figure('name', figureName, 'DefaultAxesFontSize', 10, 'units','normalized','outerposition', outerPosition)
    set(gcf,'color','w')

    for i = 1:legCount
        EEselection = EEnames(i,:);
        subplot(2,2,i)
        bar(categoryLabels, elecEnergy.(EEselection), 'stacked');
        legend(jointLabels,'Location','southoutside','Orientation','horizontal')
        title((['Electrical Energy Comparison ', EEselection]), 'FontSize', fontSize);
        ylabel('Energy [J/cycle]');
    end
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end
end