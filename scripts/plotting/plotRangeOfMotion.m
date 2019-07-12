function [] = plotRangeOfMotion(robotClass, task)

legCount = robotClass.(task).basicProperties.legCount;
EEnames  = robotClass.(task).basicProperties.EEnames;

%% get reachable positions
reachablePositions = getRangeofMotion(robotClass, task);

%% Plot mean x vs z position with reachable positions

if legCount == 1
    subplotRows = 1;
    subplotColumns = 1;
elseif legCount == 2
    subplotRows = 1;
    subplotColumns = 2;
elseif legCount > 2
    subplotRows = 2;
    subplotColumns = 2;   
end

for i = 1:legCount
    EEselection = EEnames(i,:);
    figure(7)
    set(gcf,'color','w')
    subplot(subplotRows,subplotColumns,i)
    hold on
    plot(reachablePositions.(EEselection)(:,1),reachablePositions.(EEselection)(:,2), 'color', [0.5843 0.8157 0.9882])
    plot(robotClass.(task).(EEselection).r.EEdes(:,1), robotClass.(task).(EEselection).r.EEdes(:,3), 'k', 'LineWidth', 2)
    plot(0,0,'o')
    axis equal
    xlabel('x position [m]')
    ylabel('z position [m]')
    title(['Range of motion ', EEselection])
    hold off
end
export_fig results.pdf -nocrop -append

