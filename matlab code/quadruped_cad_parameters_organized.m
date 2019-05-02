%% ANYmal clone
%Units are mm, °, and kg/m^3

%Body Dimensions - body2 is for inertia tensor, body shell is for
%visualization
anymal.body2_x.value = 50;                
anymal.body2_y.value = 117.2; %ANYmal has foot at 190mm                
anymal.body2_z.value = 344;  
anymal.body_shell_x.value = 534;
anymal.body_shell_y.value = 230.4;
anymal.body_shell_z.value = 259.2;


%Hip Dimensions
anymal.hip_y.value = 0;    
anymal.hip_z.value = 100;    

%Leg Dimensions
anymal.thigh_length.value = 332.34;           
anymal.min_thigh_width.value = 33;
anymal.shank_length.value = 332.34;           
anymal.min_shank_width.value = 30;
anymal.knee_diameter.value = 80; %outer diameter
anymal.shoulder_diameter.value = 100; %outer diameter should be larger than 
anymal.foot_diameter.value = 64; % must be larger than min_shank_width
anymal.actuator_diameter.value = 52; 
anymal.thigh_thickness.value = 33;
anymal.shank_thickness.value = 30;

%Payload Dimensions
anymal.payload_x.value = 100;
anymal.payload_y.value = 100;
anymal.payload_z.value = 100;

%Component Densities
anymal.payload_density.value = 10000;
anymal.body_density.value = 10.647*1000^3/(anymal.body2_x.value*anymal.body2_y.value*anymal.body2_z.value);          
anymal.hip_actuator_density.value = 5122.497687;
anymal.knee_actuator_density.value = 0;%7294.601558;
anymal.leg_density.value = 1162.923064;          

%Assembly constraints
anymal.front_thigh_angle.value = 45;
anymal.front_shank_angle.value = 45;
anymal.rear_thigh_angle.value = 135;
anymal.rear_shank_angle.value = 45;
anymal.lateral_leg_offset.value = 0; %offset for collision prevention during gallop
anymal.body_height.value = 420;
anymal.shoulder_x_position.value = 340;
anymal.shoulder_y_position.value = 190;
anymal.payload_x_offset.value = 0;

%Units
anymal.body2_x.unit = 'mm';
anymal.body2_y.unit = 'mm';
anymal.body2_z.unit = 'mm';
% the shell is just for visualization, it does not have any mass
anymal.body_shell_x.unit = 'mm';
anymal.body_shell_y.unit = 'mm';
anymal.body_shell_z.unit = 'mm';
anymal.hip_y.unit = 'mm';
anymal.hip_z.unit = 'mm';
anymal.thigh_length.unit = 'mm';
anymal.min_thigh_width.unit = 'mm';
anymal.shank_length.unit = 'mm';
anymal.min_shank_width.unit = 'mm';
anymal.knee_diameter.unit = 'mm';
anymal.shoulder_diameter.unit = 'mm';
anymal.foot_diameter.unit = 'mm';
anymal.actuator_diameter.unit = 'mm';
anymal.thigh_thickness.unit = 'mm';
anymal.shank_thickness.unit = 'mm';
anymal.payload_x.unit = 'mm';
anymal.payload_y.unit = 'mm';
anymal.payload_z.unit = 'mm';
anymal.payload_density.unit = 'kg/m^3';
anymal.body_density.unit = 'kg/m^3';
anymal.hip_actuator_density.unit = 'kg/m^3';
anymal.knee_actuator_density.unit = 'kg/m^3';
anymal.leg_density.unit = 'kg/m^3';
anymal.front_thigh_angle.unit = '°';
anymal.front_shank_angle.unit = '°';
anymal.rear_thigh_angle.unit = '°';
anymal.rear_shank_angle.unit = '°';
anymal.lateral_leg_offset.unit = 'mm';
anymal.body_height.unit = 'mm';
anymal.shoulder_x_position.unit = 'mm';
anymal.shoulder_y_position.unit = 'mm';
anymal.payload_x_offset.unit = 'mm';


%% Universal
%Units are mm, °, and kg/m^3

%Body Dimensions
universal.body2_x.value = 50;                
universal.body2_y.value = 117.2;                
universal.body2_z.value = 344/2;  
universal.body_shell_x.value = 534;
universal.body_shell_y.value = 230.4;
universal.body_shell_z.value = 259.2;

%Hip Dimensions
universal.hip_y.value = 0;    
universal.hip_z.value = 100;    

%Leg Dimensions
universal.thigh_length.value = 332.34;           
universal.min_thigh_width.value = 33;
universal.shank_length.value = 332.34;           
universal.min_shank_width.value = 30;
universal.knee_diameter.value = 80; %outer diameter
universal.shoulder_diameter.value = 100; %outer diameter should be larger than 
universal.foot_diameter.value = 64; % must be larger than min_shank_width
universal.actuator_diameter.value = 52; 
universal.thigh_thickness.value = 33;
universal.shank_thickness.value = 30;

%Payload Dimensions
universal.payload_x.value = 100;
universal.payload_y.value = 100;
universal.payload_z.value = 100;

%Component Densities
universal.payload_density.value = 10000;
universal.body_density.value = 10.647*1000^3/(universal.body2_x.value*universal.body2_y.value*universal.body2_z.value);          
universal.hip_actuator_density.value = 4353.639791;
universal.knee_actuator_density.value = 0;
universal.leg_density.value = 1162.923064;    
universal.body_shell_density.value = 0;

%Assembly constraints
universal.front_thigh_angle.value = 45;
universal.front_shank_angle.value = 45;
universal.rear_thigh_angle.value = 135;
universal.rear_shank_angle.value = 45;
universal.lateral_leg_offset.value = 0; %offset for collision prevention during gallop
universal.body_height.value = 50;
universal.shoulder_x_position.value = 340;
universal.shoulder_y_position.value = 190;
universal.payload_x_offset.value = 0;

%Units
universal.body2_x.unit = 'mm';
universal.body2_y.unit = 'mm';
universal.body2_z.unit = 'mm';
% the shell is just for visualization, it does not have any mass
universal.body_shell_x.unit = 'mm';
universal.body_shell_y.unit = 'mm';
universal.body_shell_z.unit = 'mm';
universal.hip_y.unit = 'mm';
universal.hip_z.unit = 'mm';
universal.thigh_length.unit = 'mm';
universal.min_thigh_width.unit = 'mm';
universal.shank_length.unit = 'mm';
universal.min_shank_width.unit = 'mm';
universal.knee_diameter.unit = 'mm';
universal.shoulder_diameter.unit = 'mm';
universal.foot_diameter.unit = 'mm';
universal.actuator_diameter.unit = 'mm';
universal.thigh_thickness.unit = 'mm';
universal.shank_thickness.unit = 'mm';
universal.payload_x.unit = 'mm';
universal.payload_y.unit = 'mm';
universal.payload_z.unit = 'mm';
universal.payload_density.unit = 'kg/m^3';
universal.body_density.unit = 'kg/m^3';
universal.hip_actuator_density.unit = 'kg/m^3';
universal.knee_actuator_density.unit = 'kg/m^3';
universal.leg_density.unit = 'kg/m^3';
universal.front_thigh_angle.unit = '°';
universal.front_shank_angle.unit = '°';
universal.rear_thigh_angle.unit = '°';
universal.rear_shank_angle.unit = '°';
universal.lateral_leg_offset.unit = 'mm';
universal.body_height.unit = 'mm';
universal.shoulder_x_position.unit = 'mm';
universal.shoulder_y_position.unit = 'mm';
universal.body_shell_density.unit = 'kg/m^3';
universal.payload_x_offset.unit = 'mm';


%% Speedy
%Units are mm, °, and kg/m^3

%Body Dimensions
speedy.body2_x.value = 50;                
speedy.body2_y.value = 117.2;                
speedy.body2_z.value = 344/4;  
speedy.body_shell_x.value = 450;
speedy.body_shell_y.value = 160;
speedy.body_shell_z.value = universal.body_shell_z.value;

%Hip Dimensions
speedy.hip_y.value = 0;    
speedy.hip_z.value = 100;    

%Leg Dimensions
speedy.thigh_length.value = 332.34;           
speedy.min_thigh_width.value = 33;
speedy.shank_length.value = 332.34;           
speedy.min_shank_width.value = 30;
speedy.knee_diameter.value = 80; %outer diameter
speedy.shoulder_diameter.value = 100; %outer diameter should be larger than 
speedy.foot_diameter.value = 64; % must be larger than min_shank_width
speedy.actuator_diameter.value = 52; 
speedy.thigh_thickness.value = 33/2.65;
speedy.shank_thickness.value = 20/2.65;

%Payload Dimensions
speedy.payload_x.value = 10;
speedy.payload_y.value = 10;
speedy.payload_z.value = 10;

%Component Densities
speedy.payload_density.value = 10000;
speedy.body_density.value = 5.3*1000^3/(speedy.body2_x.value*speedy.body2_y.value*speedy.body2_z.value);;          
speedy.hip_actuator_density.value = 4619.622521;
speedy.knee_actuator_density.value = 0;
speedy.leg_density.value = 2*1264.183969;          

%Assembly constraints
speedy.front_thigh_angle.value = 45;
speedy.front_shank_angle.value = 45;
speedy.rear_thigh_angle.value = 135;
speedy.rear_shank_angle.value = 45;
speedy.lateral_leg_offset.value = 40; %offset for collision prevention during gallop
speedy.body_height.value = 70;
speedy.shoulder_x_position.value = 310; %define front left shoulder position
speedy.shoulder_y_position.value = 100;
speedy.payload_x_offset.value = 0;

%Units
speedy.body2_x.unit = 'mm';
speedy.body2_y.unit = 'mm';
speedy.body2_z.unit = 'mm';
% the shell is just for visualization, it does not have any mass
speedy.body_shell_x.unit = 'mm';
speedy.body_shell_y.unit = 'mm';
speedy.body_shell_z.unit = 'mm';
speedy.hip_y.unit = 'mm';
speedy.hip_z.unit = 'mm';
speedy.thigh_length.unit = 'mm';
speedy.min_thigh_width.unit = 'mm';
speedy.shank_length.unit = 'mm';
speedy.min_shank_width.unit = 'mm';
speedy.knee_diameter.unit = 'mm';
speedy.shoulder_diameter.unit = 'mm';
speedy.foot_diameter.unit = 'mm';
speedy.actuator_diameter.unit = 'mm';
speedy.thigh_thickness.unit = 'mm';
speedy.shank_thickness.unit = 'mm';
speedy.payload_x.unit = 'mm';
speedy.payload_y.unit = 'mm';
speedy.payload_z.unit = 'mm';
speedy.payload_density.unit = 'kg/m^3';
speedy.body_density.unit = 'kg/m^3';
speedy.hip_actuator_density.unit = 'kg/m^3';
speedy.knee_actuator_density.unit = 'kg/m^3';
speedy.leg_density.unit = 'kg/m^3';
speedy.front_thigh_angle.unit = '°';
speedy.front_shank_angle.unit = '°';
speedy.rear_thigh_angle.unit = '°';
speedy.rear_shank_angle.unit = '°';
speedy.lateral_leg_offset.unit = 'mm';
speedy.body_height.unit = 'mm';
speedy.shoulder_x_position.unit = 'mm';
speedy.shoulder_y_position.unit = 'mm';
speedy.payload_x_offset.unit = 'mm';


%% Mini
%Units are mm, °, and kg/m^3

%Body Dimensions
mini.body2_x.value = 300;                
mini.body2_y.value = 170; %Define body used for inertia tensor                
mini.body2_z.value = 50;  
mini.body_shell_x.value = 300;
mini.body_shell_y.value = 170;
mini.body_shell_z.value = 50;

%Hip Dimensions
mini.hip_y.value = 0;    
mini.hip_z.value = 60;    

%Leg Dimensions
mini.thigh_length.value = 140;           
mini.min_thigh_width.value = 15;
mini.shank_length.value = 140;           
mini.min_shank_width.value = 10;
mini.knee_diameter.value = 40; %outer diameter. inner diameter is half this value
mini.shoulder_diameter.value = 45;  
mini.foot_diameter.value = 20; % must be larger than min_shank_width
mini.actuator_diameter.value = 30; 
mini.thigh_thickness.value = 20;
mini.shank_thickness.value = 15;

%Payload Dimensions
mini.payload_x.value = 1;
mini.payload_y.value = 1;
mini.payload_z.value = 1;

%Component Densities
mini.payload_density.value = 10000;
mini.body_density.value = 941.1764706;          
mini.hip_actuator_density.value = 14949.9751;
mini.knee_actuator_density.value = 0;
mini.leg_density.value = 5687.051164;          

%Assembly constraints
mini.front_thigh_angle.value = 45;
mini.front_shank_angle.value = 45;
mini.rear_thigh_angle.value = 135;
mini.rear_shank_angle.value = 45;
mini.lateral_leg_offset.value = 0; %offset for collision prevention during gallop
mini.body_height.value = 0; %above shoulder joint
mini.shoulder_x_position.value = 180; %define front left shoulder position
mini.shoulder_y_position.value = 100;
mini.payload_x_offset.value = 0;

%Units
mini.body2_x.unit = 'mm';
mini.body2_y.unit = 'mm';
mini.body2_z.unit = 'mm';
% the shell is just for visualization, it does not have any mass
mini.body_shell_x.unit = 'mm';
mini.body_shell_y.unit = 'mm';
mini.body_shell_z.unit = 'mm';
mini.hip_y.unit = 'mm';
mini.hip_z.unit = 'mm';
mini.thigh_length.unit = 'mm';
mini.min_thigh_width.unit = 'mm';
mini.shank_length.unit = 'mm';
mini.min_shank_width.unit = 'mm';
mini.knee_diameter.unit = 'mm';
mini.shoulder_diameter.unit = 'mm';
mini.foot_diameter.unit = 'mm';
mini.actuator_diameter.unit = 'mm';
mini.thigh_thickness.unit = 'mm';
mini.shank_thickness.unit = 'mm';
mini.payload_x.unit = 'mm';
mini.payload_y.unit = 'mm';
mini.payload_z.unit = 'mm';
mini.payload_density.unit = 'kg/m^3';
mini.body_density.unit = 'kg/m^3';
mini.hip_actuator_density.unit = 'kg/m^3';
mini.knee_actuator_density.unit = 'kg/m^3';
mini.leg_density.unit = 'kg/m^3';
mini.front_thigh_angle.unit = '°';
mini.front_shank_angle.unit = '°';
mini.rear_thigh_angle.unit = '°';
mini.rear_shank_angle.unit = '°';
mini.lateral_leg_offset.unit = 'mm';
mini.body_height.unit = 'mm';
mini.shoulder_x_position.unit = 'mm';
mini.shoulder_y_position.unit = 'mm';
mini.payload_x_offset.unit = 'mm';


%% Massivo
%Units are mm, °, and kg/m^3

%Body Dimensions
massivo.body2_x.value = 600;                
massivo.body2_y.value = 500; %Define body used for inertia tensor                
massivo.body2_z.value = 200;  
massivo.body_shell_x.value = 600;
massivo.body_shell_y.value = 500;
massivo.body_shell_z.value = 200;

%Hip Dimensions
massivo.hip_y.value = 0;    
massivo.hip_z.value = 200;    

%Leg Dimensions
massivo.thigh_length.value = 260;           
massivo.min_thigh_width.value = 50;
massivo.shank_length.value = 260;           
massivo.min_shank_width.value = 40;
massivo.knee_diameter.value = 120; %outer diameter. inner diameter is half this value
massivo.shoulder_diameter.value = 180;  
massivo.foot_diameter.value = 50; % must be larger than min_shank_width
massivo.actuator_diameter.value = 120; 
massivo.thigh_thickness.value = 50;
massivo.shank_thickness.value = 40;

%Payload Dimensions
massivo.payload_x.value = 400;
massivo.payload_y.value = 300;
massivo.payload_z.value = 200;

%Component Densities
massivo.payload_density.value = 30*1000^3/(massivo.payload_x.value*massivo.payload_y.value*massivo.payload_z.value);
massivo.body_density.value = 166.6666667;          
massivo.hip_actuator_density.value = 1157.850836;
massivo.knee_actuator_density.value = 0;
massivo.leg_density.value = 2182.073048;          

%Assembly constraints
massivo.front_thigh_angle.value = 70;
massivo.front_shank_angle.value = 90;
massivo.rear_thigh_angle.value = 110;
massivo.rear_shank_angle.value = 90;
massivo.lateral_leg_offset.value = 0; %offset for collision prevention during gallop
massivo.body_height.value = 0; %above shoulder joint
massivo.shoulder_x_position.value = 730/2; %define front left shoulder position
massivo.shoulder_y_position.value = 300;
massivo.payload_x_offset.value = 0;

%Units
massivo.body2_x.unit = 'mm';
massivo.body2_y.unit = 'mm';
massivo.body2_z.unit = 'mm';
% the shell is just for visualization, it does not have any mass
massivo.body_shell_x.unit = 'mm';
massivo.body_shell_y.unit = 'mm';
massivo.body_shell_z.unit = 'mm';
massivo.hip_y.unit = 'mm';
massivo.hip_z.unit = 'mm';
massivo.thigh_length.unit = 'mm';
massivo.min_thigh_width.unit = 'mm';
massivo.shank_length.unit = 'mm';
massivo.min_shank_width.unit = 'mm';
massivo.knee_diameter.unit = 'mm';
massivo.shoulder_diameter.unit = 'mm';
massivo.foot_diameter.unit = 'mm';
massivo.actuator_diameter.unit = 'mm';
massivo.thigh_thickness.unit = 'mm';
massivo.shank_thickness.unit = 'mm';
massivo.payload_x.unit = 'mm';
massivo.payload_y.unit = 'mm';
massivo.payload_z.unit = 'mm';
massivo.payload_density.unit = 'kg/m^3';
massivo.body_density.unit = 'kg/m^3';
massivo.hip_actuator_density.unit = 'kg/m^3';
massivo.knee_actuator_density.unit = 'kg/m^3';
massivo.leg_density.unit = 'kg/m^3';
massivo.front_thigh_angle.unit = '°';
massivo.front_shank_angle.unit = '°';
massivo.rear_thigh_angle.unit = '°';
massivo.rear_shank_angle.unit = '°';
massivo.lateral_leg_offset.unit = 'mm';
massivo.body_height.unit = 'mm';
massivo.shoulder_x_position.unit = 'mm';
massivo.shoulder_y_position.unit = 'mm';
massivo.payload_x_offset.unit = 'mm';

%% Centaur
%Units are mm, °, and kg/m^3

%Body Dimensions
centaur.body2_x.value = 600;                
centaur.body2_y.value = 500; %Define body used for inertia tensor                
centaur.body2_z.value = 200;  
centaur.body_shell_x.value = 600;
centaur.body_shell_y.value = 500;
centaur.body_shell_z.value = 200;

%Hip Dimensions
centaur.hip_y.value = 0;    
centaur.hip_z.value = 200;    

%Leg Dimensions
centaur.thigh_length.value = 260;           
centaur.min_thigh_width.value = 50;
centaur.shank_length.value = 260;           
centaur.min_shank_width.value = 40;
centaur.knee_diameter.value = 120; %outer diameter. inner diameter is half this value
centaur.shoulder_diameter.value = 180;  
centaur.foot_diameter.value = 50; % must be larger than min_shank_width
centaur.actuator_diameter.value = 120; 
centaur.thigh_thickness.value = 50;
centaur.shank_thickness.value = 40;

%Payload Dimensions
centaur.payload_x.value = 200;
centaur.payload_y.value = 300;
centaur.payload_z.value = 400;

%Component Densities
centaur.payload_density.value = 30*1000^3/(massivo.payload_x.value*massivo.payload_y.value*massivo.payload_z.value);
centaur.body_density.value = 166.6666667;          
centaur.hip_actuator_density.value = 1157.850836;
centaur.knee_actuator_density.value = 0;
centaur.leg_density.value = 2182.073048;          

%Assembly constraints
centaur.front_thigh_angle.value = 70;
centaur.front_shank_angle.value = 90;
centaur.rear_thigh_angle.value = 110;
centaur.rear_shank_angle.value = 90;
centaur.lateral_leg_offset.value = 0; %offset for collision prevention during gallop
centaur.body_height.value = 0; %above shoulder joint
centaur.shoulder_x_position.value = 730/2; %define front left shoulder position
centaur.shoulder_y_position.value = 300;
centaur.payload_x_offset.value = 0.5*(centaur.body2_x.value + 0.5*centaur.payload_x.value);

%Units
centaur.body2_x.unit = 'mm';
centaur.body2_y.unit = 'mm';
centaur.body2_z.unit = 'mm';
% the shell is just for visualization, it does not have any mass
centaur.body_shell_x.unit = 'mm';
centaur.body_shell_y.unit = 'mm';
centaur.body_shell_z.unit = 'mm';
centaur.hip_y.unit = 'mm';
centaur.hip_z.unit = 'mm';
centaur.thigh_length.unit = 'mm';
centaur.min_thigh_width.unit = 'mm';
centaur.shank_length.unit = 'mm';
centaur.min_shank_width.unit = 'mm';
centaur.knee_diameter.unit = 'mm';
centaur.shoulder_diameter.unit = 'mm';
centaur.foot_diameter.unit = 'mm';
centaur.actuator_diameter.unit = 'mm';
centaur.thigh_thickness.unit = 'mm';
centaur.shank_thickness.unit = 'mm';
centaur.payload_x.unit = 'mm';
centaur.payload_y.unit = 'mm';
centaur.payload_z.unit = 'mm';
centaur.payload_density.unit = 'kg/m^3';
centaur.body_density.unit = 'kg/m^3';
centaur.hip_actuator_density.unit = 'kg/m^3';
centaur.knee_actuator_density.unit = 'kg/m^3';
centaur.leg_density.unit = 'kg/m^3';
centaur.front_thigh_angle.unit = '°';
centaur.front_shank_angle.unit = '°';
centaur.rear_thigh_angle.unit = '°';
centaur.rear_shank_angle.unit = '°';
centaur.lateral_leg_offset.unit = 'mm';
centaur.body_height.unit = 'mm';
centaur.shoulder_x_position.unit = 'mm';
centaur.shoulder_y_position.unit = 'mm';
centaur.payload_x_offset.unit = 'mm';

%% Save the results to import to NX

quadruped_geometry = anymal;
save('anymal','quadruped_geometry');
clear quadruped_geometry;

quadruped_geometry = universal;
save('universal','quadruped_geometry');
clear quadruped_geometry;

quadruped_geometry = speedy;
save('speedy','quadruped_geometry');
clear quadruped_geometry;

quadruped_geometry = mini;
save('mini','quadruped_geometry');
clear quadruped_geometry;

quadruped_geometry = massivo;
save('massivo','quadruped_geometry');
clear quadruped_geometry;

quadruped_geometry = centaur;
save('centaur','quadruped_geometry');
clear quadruped_geometry;

writeGeometry2File('anymal.mat', '\\d\users\all\chamicha\Thesis\CAD Models\expressions\');
writeGeometry2File('universal.mat', '\\d\users\all\chamicha\Thesis\CAD Models\expressions\');
writeGeometry2File('speedy.mat', '\\d\users\all\chamicha\Thesis\CAD Models\expressions\');
writeGeometry2File('mini.mat', '\\d\users\all\chamicha\Thesis\CAD Models\expressions\');
writeGeometry2File('massivo.mat', '\\d\users\all\chamicha\Thesis\CAD Models\expressions\');
writeGeometry2File('centaur.mat', '\\d\users\all\chamicha\Thesis\CAD Models\expressions\');

