function grasp(gen3Kinova)

pause(1)
toolCommand = int32(3);    % position control mode
toolDuration = 0;
toolCmd = 0.55;
isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
if isOk
    disp('Command sent to the gripper. Wait for the gripper to close.')
else
    error('Command Error.');
end
pause(1)

end