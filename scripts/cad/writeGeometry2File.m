function [ ] = writeGeometry2File(input_file, save_path)

    % Load workspace from Mat file
    load(input_file); % contains struct: robotGeometry
    
    % Extract file name
    [~,input_name,~] = fileparts(input_file);
        
    % Create new file name
    filename = [input_name '.exp'];

    % Open file
    fileID = fopen([save_path filename], 'w');
      
    % Write all variables of optimal_geometry to file
    robotFieldnames = fieldnames(robot);
    for i = 1:length(robotFieldnames)        
        var_name = robotFieldnames{i};
        var_unit = robot.(var_name).unit;
        var_value = robot.(var_name).value;

        if strcmp(var_unit, 'int')
            fprintf(fileID,'(Integer) %s=%i\n', var_name, var_value);
        elseif strcmp(var_unit, 'm')
            % Convert m to mm for NX
            var_unit = 'mm';
            var_value = var_value * 1000;
            fprintf(fileID,'[%s]%s=%0.2f\n', var_unit, var_name, var_value);
        elseif strcmp(var_unit, 'rad')
            % Convert rad to deg for NX
            var_unit = '°';
            var_value = rad2deg(var_value);
            fprintf(fileID,'[%s]%s=%0.2f\n', var_unit, var_name, var_value);
            %Check what units NX wants for density
        elseif strcmp(var_unit, 'kg/m^3')
            fprintf(fileID,'[%s]%s=%0.2f\n', var_unit, var_name, var_value);
        elseif strcmp(var_unit, 'mm')
            fprintf(fileID,'[%s]%s=%0.2f\n', var_unit, var_name, var_value);
        elseif strcmp(var_unit, '°')
            fprintf(fileID,'[%s]%s=%0.2f\n', var_unit, var_name, var_value);
        elseif strcmp(var_unit, 'kg')
            fprintf(fileID,'[%s]%s=%0.2f\n', var_unit, var_name, var_value);
        else
            warning(['Failed to write: ' var_name ' variable.']);
        end            
    end
    
    fclose(fileID);
    
    quadruped_name = convertCharsToStrings(filename);
    disp(quadruped_name, 'written to file');   

end
