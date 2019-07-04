function terrainMap = constructTerrainMap(EE, legCount, EEnames)
    terrainMap = [];

    for i = 1:legCount
        k = 0;
        EEselection = EEnames(i,:);
        for j = 1:length(EE.(EEselection).position)
            if EE.(EEselection).force(j,3) ~= 0 % foot in contact with ground
                k = k+1;
                terrainMapTemp.(EEselection)(k,1) = EE.(EEselection).position(j,1); % x position
                terrainMapTemp.(EEselection)(k,2) = EE.(EEselection).position(j,2); % y position
                terrainMapTemp.(EEselection)(k,3) = EE.(EEselection).position(j,3); % z position
            end
        end
        % concatenate terrain maps from each leg to create surface
        terrainMap = [terrainMap; terrainMapTemp.(EEselection)];
    end
    
    % round values to nearest cm
    terrainMap = round(terrainMap,2);
    % remove duplicate rows so we only get one point for each stance phase
    terrainMap = unique(terrainMap, 'rows');
end