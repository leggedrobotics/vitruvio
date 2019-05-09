%% get joint velocities using finite difference

for i =1:length(q.LF)-1
    omegaFiniteDifference.LF(i,:) = (q.LF(i+1,:) - q.LF(i,:)) /dt;
    omegaFiniteDifference.LH(i,:) = (q.LF(i+1,:) - q.LH(i,:)) /dt;
    omegaFiniteDifference.RF(i,:) = (q.LF(i+1,:) - q.RF(i,:)) /dt;
    omegaFiniteDifference.RH(i,:) = (q.LF(i+1,:) - q.RH(i,:)) /dt;
end
