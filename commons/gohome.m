function gohome(gen3Kinova, pause_time)

jointCmd = wrapTo360([0 0 0 0 0 0]);
constraintType = int32(0);
speed = 0;
duration = 0;

isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
if isOk
    disp('success');
else
    disp('SendJointAngles   cmd error');
    return;
end

status = 1;
% Check if the robot has reached the end postion
while status
    [isOk, ~, ~, ~] = gen3Kinova.SendRefreshFeedback();
    if isOk
        [~, status] = gen3Kinova.GetMovementStatus();
    else
        error('SendRefreshFeedback error');
    end
end

pause(pause_time)
ungrasp(gen3Kinova)

end