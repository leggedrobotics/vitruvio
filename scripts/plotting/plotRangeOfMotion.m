function [] = plotRangeOfMotion(data, saveFiguresToPDF)

    legCount = data.basicProperties.legCount;
    EEnames  = data.basicProperties.EEnames;
    if saveFiguresToPDF
        outerPosition = [0 0 1 1]; % Fullscreen
    else
        outerPosition = [0.5 0.5 0.5 0.5]; % Top right corner
    end
    
    %% Get reachable positions
    reachablePositions = getRangeofMotion(data);

    %% Plot mean x vs z position with reachable positions

    subplotRows = ceil(legCount/2);
    if legCount < 2
        subplotColumns = 1;
    else
        subplotColumns = 2;
    end

    figure('name', 'Range of Motion', 'units','normalized','outerposition',outerPosition)
    set(gcf,'color','w')
    for i = 1:legCount
        EEselection = EEnames(i,:);
        set(gcf,'color','w')
        subplot(subplotRows,subplotColumns,i)
        hold on
        plot(reachablePositions.(EEselection)(:,1),reachablePositions.(EEselection)(:,2), 'color', [0.5843 0.8157 0.9882])
        plot(data.(EEselection).r.EEdes(:,1), data.(EEselection).r.EEdes(:,3), 'k', 'LineWidth', 1)

        plot(data.(EEselection).r.HAA(1,1), data.(EEselection).r.HAA(1,3),'ko')
        text(data.(EEselection).r.HAA(1,1), data.(EEselection).r.HAA(1,3)+0.02, 'HAA', 'fontSize', 14) 
        
        plot(data.(EEselection).r.HFE(1,1), data.(EEselection).r.HFE(1,3), 'ko')
        text(data.(EEselection).r.HFE(1,1), data.(EEselection).r.HFE(1,3)+0.02, 'HFE', 'fontSize', 14) 
        hold off
        axis equal
        xlabel('x position [m]')
        ylabel('z position [m]')
        title(['Range of motion ', EEselection])
    end
    if saveFiguresToPDF
        export_fig results.pdf -nocrop -append
    end
end