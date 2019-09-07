% Enter robot class and task
robot = 'vitruvianBiped';
task  = 'pushupSquat2';
loopMotion = false; % motion is repeated three times
taskOpt = strcat(task,'Opt');

% Add 2*pi to joint position qHFE to keep it in the positive region
actuatorq    = results.(robot).(task).RF.actuatorq;
actuatorqOpt = results.(robot).(task).RF.actuatorqOpt;

actuatorq(:,2)    = 2*pi + results.(robot).(task).RF.actuatorq(:,2);
actuatorqOpt(:,2) = 2*pi + results.(robot).(task).RF.actuatorqOpt(:,2);

j = 0;
for i = 1:length(actuatorq(:,1))
    if mod(i-1,1)==0 || i==1% Save every xth term
        j = j+1;
        time(j,1) = results.(robot).(task).time(i,1);
        q(j,:)    = round(actuatorq(i,2:3),3); % round to actuator precision
        qOpt(j,:) = round(actuatorqOpt(i,2:3),3); % round to actuator precision
    end
end

% Loop motion 3 times
if loopMotion
    q = [q; q; q];
    qOpt = [qOpt; qOpt; qOpt];
    time = [time; time+time(end); time+2*time(end)];
end

delayStartTime = 2;
s.time = time+delayStartTime; % two second delay before beginning motion
s.q    = q;

t.time = time+delayStartTime;
t.q    = qOpt;

struct2csv(s, task)
struct2csv(t, taskOpt)