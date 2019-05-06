figure()

% h = animatedline;
% 
% for i = 1:length(r.LF.KFE)
%     addpoints(h, r.LF.KFE(i,1),r.LF.KFE(i,3),'o');
%     drawnow
%     pause(0.3)
% end
i = 1
% plot(r.LF.HAA(:,1),r.LF.HAA(:,3), ...
%      r.LF.HFE(:,1),r.LF.HFE(:,3), ...
%      r.LF.KFE(:,1),r.LF.KFE(:,3), ...
%      r.LF.EE(:,1),r.LF.EE(:,3))

for i = 1:length(r.LF.KFE)
hold on
% scatter(r.LF.EE(i,1),r.LF.EE(i,3))
% scatter(r.LF.KFE(i,1),r.LF.KFE(i,3))
% scatter(r.LF.HFE(i,1),r.LF.HFE(i,3))

plot([r.LF.KFE(i,1),r.LF.KFE(i,3)],[r.LF.EE(i,1),r.LF.EE(i,3)])
hold off
pause(0.3)
end


