figure()
axis equal
title('ANYmal slow trot')
xlabel('EE position x [m]');
ylabel('EE position z [m]');
rEExInterp = ANYmal.slowTrotGoodMotionBadForce.LF.r.EEdes(:,1);
rEEzInterp = ANYmal.slowTrotGoodMotionBadForce.LF.r.EEdes(:,3);
rEEx = ANYmalNoInterpolation.slowTrotGoodMotionBadForce.LF.r.EEdes(:,1);
rEEz = ANYmalNoInterpolation.slowTrotGoodMotionBadForce.LF.r.EEdes(:,3);
subplot(2,1,1)

scatter(rEEx, rEEz)
title('ANYmal slow trot')
xlabel('EE position x [m]');
ylabel('EE position z [m]');
subplot(2,1,2)
scatter(rEExInterp, rEEzInterp)
xlabel('EE position x [m]');
ylabel('EE position z [m]');
