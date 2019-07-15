function  q0 = getInitialJointAnglesForDesiredConfig(EEselection, configSelection)
    % The inverse kinematics algorithm is initialized with these angles and
    % finds the closest solution. The angles have been selected such that the
    % inverse kinematics reliably converges to the desired leg layout with
    % knees forward or backward for the modeled robots. If your robot diverges
    % too far from the nominal design, the desired layout may no longer be
    % obtained. In this case the initial angles here should be updated to
    % correspond more closely to the initial angles for your robot in the
    % desired layout.

    kneesForward = [0  0.64 -4.4  0];
    kneesBackward = [0  -3.8 -0.66 0];

    % If the robot has fewer than four leg, the additional values returned here
    % are redundant and do not affect the solution.
    q0.X.LF = kneesBackward;
    q0.X.RF = kneesBackward;
    q0.X.LH = kneesForward;
    q0.X.RH = kneesForward;

    q0.M.LF = kneesBackward;
    q0.M.RF = kneesBackward;
    q0.M.LH = kneesBackward;
    q0.M.RH = kneesBackward;

    q0 = q0.(configSelection).(EEselection);
end