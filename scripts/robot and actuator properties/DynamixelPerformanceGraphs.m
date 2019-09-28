function [p64R, torque64R, pXM540, torqueXM540, speed64R, speedXM540, efficiency64R, efficiencyXM540] = DynamixelPerformanceGraphs
% Returns polynomials describing the maximum speed at each torque value
%% 64R
maxTorqueLimit = 7.3;  % [Nm]   
maxqdotLimit   = 6.6;  % [rad/s] 

% input torque speed points from performance graph
torque64R     = [0   0.135 0.3    0.6    0.9    1.2    1.5    1.8    2.1    2.4    2.7    2.85   7.3]; %Nm
speed64R      = [6.6 6.57  6.5450 6.0737 5.7596 5.2360 4.8171 4.4506 3.8746 3.4558 3.0369 2.6180 0]; %rad/s
efficiency64R = [[]  29    42     49     49     47     44     40     35     31     26     24     []]; % percent
current64R    = []; %A

p64R = polyfit(torque64R, speed64R, 3);
y64R = polyval(p64R,torque64R);

%% XM540
% input torque speed points from performance graph
torqueXM540     = [0    0.4    1      2      3      4      5      6      7      8      8.6    10.6];
speedXM540      = [3.14 3.0159 2.8903 2.6180 2.3038 2.0420 1.6755 1.3823 0.9425 0.5236 0.2094 0];
efficiencyXM540 = [[]   28     40     46     44     40     34     28     20     12     5       []]; % percent
currentXM540    = []; %A
        
pXM540 = polyfit(torqueXM540, speedXM540, 3);
yXM540 = polyval(pXM540,torqueXM540);