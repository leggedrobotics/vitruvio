%% get joint velocities and accelerations using finite difference

for i =1:length(q.LF)-1
    FiniteDifference.omega.LF(i,:) = (q.LF(i+1,:) - q.LF(i,:)) /dt;
    FiniteDifference.omega.LH(i,:) = (q.LH(i+1,:) - q.LH(i,:)) /dt;
    FiniteDifference.omega.RF(i,:) = (q.RF(i+1,:) - q.RF(i,:)) /dt;
    FiniteDifference.omega.RH(i,:) = (q.RH(i+1,:) - q.RH(i,:)) /dt;
end

for i =1:length(q.LF)-2
    FiniteDifference.alpha.LF(i,:) = (FiniteDifference.omega.LF(i+1,:) - FiniteDifference.omega.LF(i,:)) /dt;
    FiniteDifference.alpha.LH(i,:) = (FiniteDifference.omega.LH(i+1,:) - FiniteDifference.omega.LH(i,:)) /dt;
    FiniteDifference.alpha.RF(i,:) = (FiniteDifference.omega.RF(i+1,:) - FiniteDifference.omega.RF(i,:)) /dt;
    FiniteDifference.alpha.RH(i,:) = (FiniteDifference.omega.RH(i+1,:) - FiniteDifference.omega.RH(i,:)) /dt;
end

time = 0:length(FiniteDifference.omega.LF) ;
time = dt*time;

plot(time, q.LF(:,1), 'r--', ...
     time, q.LF(:,2), 'b--', ...
     time, q.LF(:,3), 'c--', ...
     time, q.RF(:,1), 'r', ...
     time, q.RF(:,2), 'b', ...
     time, q.RF(:,3), 'c')
   


