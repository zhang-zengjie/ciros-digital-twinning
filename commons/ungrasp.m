function ungrasp(gen3Kinova)

pause(1)
toolCommand = int32(2);    % Velocity control mode
toolDuration = 0;
toolCmd = 1; % Close the gripper with full speed
isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
if isOk
    disp('Command sent to the gripper. Wait for the gripper to close.')
else
    error('Command Error.');
end
pause(1)

end