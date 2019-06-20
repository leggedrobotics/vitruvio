% %% NEW METHOD If points are spread too far apart, interpolate to add more points
% allowableDeviation = 0.08; % [m]
% for j = 1:4
%         EEselection = EEnames(j,:);
%         % initialize deviation
%         [neighbouringPointDeviation.(EEselection), neighbouringPointDeviationMax.(EEselection)] = getDeviationFromNeighbouringPoint(relativeMotionHipEE, EEselection);
% end
% 
% for j = 1:4
%         EEselection = EEnames(j,:);
%         i = 1; % start at beginning of relativeMotionHipEE array
%         lengthOfPositionArray = length(relativeMotionHipEE.(EEselection).position);
%     while i < lengthOfPositionArray
%          if  neighbouringPointDeviation.(EEselection)(i) > allowableDeviation
%              % if the deviation between two points is larger than the
%              % allowable, interpolate a point between these two points. A
%              % point is added at this position for each leg.
%             [interpolatedPositionData, interpolatedVelocityData, interpolatedForceData, interpolatedBodyRotationData, newTimeData] = generateAdditionalPoints(Leg, relativeMotionHipEE, EE, C_IBody, EEnames, i);
%                 
%         % update the relativeMotion arrays with the new points
%             relativeMotionHipEE.LF.position = interpolatedPositionData.LF;
%             relativeMotionHipEE.LF.velocity = interpolatedVelocityData.LF;
%             EE.LF.force = interpolatedForceData.LF;
% 
%             relativeMotionHipEE.LH.position = interpolatedPositionData.LH;
%             relativeMotionHipEE.LH.velocity = interpolatedVelocityData.LH;
%             EE.LH.force = interpolatedForceData.LH;
% 
%             relativeMotionHipEE.RF.position = interpolatedPositionData.RF;
%             relativeMotionHipEE.RF.velocity = interpolatedVelocityData.RF;
%             EE.RF.force = interpolatedForceData.RF;        
% 
%             relativeMotionHipEE.RH.position = interpolatedPositionData.RH;
%             relativeMotionHipEE.RH.velocity = interpolatedVelocityData.RH;
%             EE.RH.force = interpolatedForceData.RH;
% 
%             % add new point to neighbouringPointDeviation vector with
%             % value equal to half of the deviation pre interpolation
%             % value
%             neighbouringPointDeviation.LF = [neighbouringPointDeviation.LF(1:i-1); neighbouringPointDeviation.LF(i)/2; neighbouringPointDeviation.LF(i)/2; neighbouringPointDeviation.LF(i+1:end)];
%             neighbouringPointDeviation.LH = [neighbouringPointDeviation.LH(1:i-1); neighbouringPointDeviation.LH(i)/2; neighbouringPointDeviation.LH(i)/2; neighbouringPointDeviation.LH(i+1:end)];
%             neighbouringPointDeviation.RF = [neighbouringPointDeviation.RF(1:i-1); neighbouringPointDeviation.RF(i)/2; neighbouringPointDeviation.RF(i)/2; neighbouringPointDeviation.RF(i+1:end)];
%             neighbouringPointDeviation.RH = [neighbouringPointDeviation.RH(1:i-1); neighbouringPointDeviation.RH(i)/2; neighbouringPointDeviation.RH(i)/2; neighbouringPointDeviation.RH(i+1:end)];
% 
%             Leg.time = newTimeData;
%             C_IBody = interpolatedBodyRotationData;
%             
%             lengthOfPositionArray = lengthOfPositionArray + 1; % increased length of position array by one by adding interpolated point
%             i = 0; % if a point is interpolated, start search again from the beginning 
%          end
%         i = i+1; % move forward to next timestep
%     end
% end
