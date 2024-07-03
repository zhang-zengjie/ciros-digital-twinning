function actuator_feedback = goto(gen3Kinova, angles, vel, acc, timestamp, ...
    t_start, old_feedback)

jointCmd = wrapTo360(angles(1,:));
constraintType = int32(0);
speed = 0;
duration = 0;
 
isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
if isOk
    disp('success');
else
    disp('SendJointAngles cmd error');
    isOk = gen3Kinova.DestroyRobotApisWrapper();

    clear
    close all
    clc
    return;
end

pause(0.05)

isOk = gen3Kinova.SendPreComputedTrajectory(angles.', vel.', acc.', timestamp, size(timestamp,2));
if isOk
    disp('SendPreComputedTrajectory success');
else
    disp('SendPreComputedTrajectory command error');
    isOk = gen3Kinova.DestroyRobotApisWrapper();

    clear
    close all
    clc
end

actuator_feedback = old_feedback; %#ok<*AGROW>
t_traj_start = toc(t_start);
while true
    t_now = toc(t_start);
    t_traj_elapsed = t_now - t_traj_start;
    if t_traj_elapsed >= (timestamp(end) + 0.05)
        disp('Trajectory execution complete');
        break
    else
        [~, ~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
        actuator_feedback = [actuator_feedback; [t_now, actuatorFb.position]];
%         [~, baseFb, ~, ~] = gen3Kinova.SendRefreshFeedback();
%         actuator_feedback = [actuator_feedback; [t_now, baseFb.tool_pose]];
    end
end

end