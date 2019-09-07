function [p1 torque64R p2 torqueXM540] = DynamixelPerformanceGraphs
% Returns polynomials describing the maximum speed at each torque value
%% 64R
maxTorqueLimit = 7.3;  % [Nm]   
maxqdotLimit   = 6.6;  % [rad/s] 

% input torque speed points from performance graph
torque64R     = [0   0.135 0.3    0.6    0.9    1.2    1.5    1.8    2.1    2.4    2.7    2.85   7.3]; %Nm
speed64R      = [6.6 6.57  6.5450 6.0737 5.7596 5.2360 4.8171 4.4506 3.8746 3.4558 3.0369 2.6180 0]; %rad/s
efficiency64R = []; 
current64R    = []; %A

p1 = polyfit(torque64R, speed64R, 3);
y1 = polyval(p1,torque64R);

%% XM540
% input torque speed points from performance graph
torqueXM540     = [0    0.4    1      2      3      4      5      6      7      8      8.6    10.6];
speedXM540      = [3.14 3.0159 2.8903 2.6180 2.3038 2.0420 1.6755 1.3823 0.9425 0.5236 0.2094 0];
efficiencyXM540 = [];
currentXM540    = []; %A
        
p2 = polyfit(torqueXM540, speedXM540, 3);
y2 = polyval(p2,torqueXM540);