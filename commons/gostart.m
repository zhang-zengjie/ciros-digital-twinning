function gostart(gen3Kinova, angles)
jointCmd = wrapTo360(angles(1,:));
constraintType = int32(0);
speed = 0;
duration = 0;

isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
if isOk
    disp('success');
else
    disp('SendJointAngles   cmd error');
    isOk = gen3Kinova.DestroyRobotApisWrapper();

    clear all
    close all
    clc
    return;
end

pause(3)
end