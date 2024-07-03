%% Initialization
clear
close all
clc

%% Include Visualization Files
addpath('commons');
addpath('models');
addpath('libs');
addpath('models/robotiq_arg85_description');
addpath('models/robotiq_arg85_description/meshes');
addpath('models/kortex_description/arms/gen3/6dof/meshes');
robot = importrobot('kinovaGen3_6DOF_V12.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.25 0.25 0.25 0.25 0.25 0.25];
initGuess = robot.homeConfiguration;
rng(0)

% Use set points in Cartesian space as a simple heuristic motion planner

disp('Generating waypoints ...');
points = [0.375, -0.36, 0.25, 0, 90, 90;
    0.355, -0.375, 0.115, 0, 90, 90;
    0.375, -0.36, 0.25, 0, 90, 90;
    -0.135, -0.6, 0.3, 0, 0, 90;
    -0.560, -0.141, 0.3, 0, -90, 90;
    -0.564, -0.111, 0.16, 0, -90, 90;
    -0.5, -0.0085, 0.5, 0,-90,90;
    -0.5, -0.0085, 0.3, 0,-90,180;
    -0.585, -0.02, 0.13, 0,-90,180;
    -0.61, -0.02, 0.13, 0,-90,180;
    -0.61, -0.02, 0.2, 0,-90,180;
    0.335, 0.4, 0.2, 0, -90, 180;
    ];

num_points = size(points, 1);
X = points(:,1);
Y = points(:,2);
Z = points(:,3);
Theta = points(:,4);
Phi = points(:,5);
Psi = points(:,6);

% Trajectory Planning
rotu = @(u, angle) cos(angle) * eye(3) ...
    + sin(angle) * [0, -u(3), u(2); u(3), 0, -u(1); -u(2), u(1), 0] ...
    + (1 - cos(angle)) * u * transpose(u);

Ndof = numel(initGuess);
config = zeros(num_points+1, Ndof);
config(1, :) = initGuess;
vel_capacity = 0.4;
acc_capacity = 0.6;
v_max = pi/180 * [78 78 78 68 68 68] * vel_capacity;
a_max = pi/180 * [295, 295, 295, 570, 570, 570] * acc_capacity;
WayPoints = zeros(1, Ndof+1);
delta = zeros(num_points, Ndof);
cruise_time = delta;
accel_time= cruise_time;
v_peak= accel_time;
dt = 0.01;
pause_steps = 0.1/dt;
num_steps= zeros(num_points+1, Ndof);
milestones = zeros(num_points, 2);

flag = 1;
for i = 2:num_points+1
    R = rotu([1;0;0], pi/180*Theta(i-1))*rotu([0;0;1], pi/180*Phi(i-1))*rotu([1;0;0], pi/180*Psi(i-1));
    P = [X(i-1); Y(i-1); Z(i-1)];
    T = [[R, P]; [0 0 0 1]];

    config(i, :) = ik('EndEffector_Link', T, weights, config(i-1, :));
    
    delta(i-1, :) = config(i, :) - config(i-1, :);
    cruise_time(i-1, :) = max( abs(delta(i-1, :))./ v_max- v_max./a_max, 0 );
    
    for j = 1:Ndof
        if cruise_time(i-1, j) == 0
            accel_time(i-1, j) = 0.5 * sqrt(2 * abs(delta(i-1, j))/a_max(j));
            v_peak(i-1, j) = a_max(j) * accel_time(i-1, j);
        else
            accel_time(i-1, j) = v_max(j)/a_max(j);
            v_peak(i-1, j) = v_max(j);
        end
        num_steps(i, j) = floor((2*accel_time(i-1, j) + cruise_time(i-1, j))/dt);
    end
    
    for j = 1:Ndof
        WayPoints(flag:flag+2*pause_steps+max(num_steps(i, :))-1, j+1) = config(i, j);
        
        [q,~,~,~,~] = trapveltraj([config(i-1, j), config(i, j)], ...
            max(num_steps(i, :)), 'PeakVelocity', v_peak(i-1, j), ...
            'Acceleration', a_max(j));
        
        WayPoints(flag:flag+pause_steps-1, j+1) = config(i-1, j);
        WayPoints(flag+pause_steps:flag+pause_steps+max(num_steps(i, :))-1, j+1) = q;
        clear q
    end
    WayPoints(flag:flag+2*pause_steps+max(num_steps(i, :))-1, 1) = ...
        (flag-1)*dt : dt : (flag+2*pause_steps+max(num_steps(i, :))-2)*dt;
    milestones(i-1, :) = [flag, flag+2*pause_steps+max(num_steps(i, :))-1];
    flag = size(WayPoints, 1) + 1;
end

% figure
% for i = 1:Ndof
%     hold on
%     plot(1:size(WayPoints,1),WayPoints(:,i+1))
% end
% hold off

tf = WayPoints(end, 1);

disp('Waypoints successfully generated!');

%% Run Simulation

disp('Initializing simulation ...');

traj_v_max = zeros(1,6);
traj_a_max = zeros(1,6);
for i = 1:num_points
    temp = WayPoints(milestones(i, 1):milestones(i, 2), :);
    
    simtime = temp(:,1) - temp(1,1);
    timestamp = 0:0.001:(temp(end,1) - temp(1,1));
    
    angles = 180/pi * temp(:, 2:Ndof+1);
    
    vel = diff(angles)/dt;
    vel(1,:) = 0;
    vel(end+1,:) = 0;
    
    acc = diff(vel)/dt;
    acc(1,:) = 0;
    acc(end+1,:) = 0;

    angles = interp1(simtime,angles,timestamp);
    vel = interp1(simtime,vel,timestamp);
    acc = interp1(simtime,acc,timestamp);
    traj_v_max = max(traj_v_max, max(abs(vel)));
    traj_a_max = max(traj_a_max, max(abs(acc)));
    
    assignin('base', ['angles', num2str(i)], angles);
    assignin('base', ['vel', num2str(i)], vel);
    assignin('base', ['acc', num2str(i)], acc);
    assignin('base', ['timestamp', num2str(i)], timestamp);
end

disp('Simulation initialized.');

%% connect to robot

disp('Preparing for connetion ...');

if ~exist('temp', 'dir')
    mkdir('temp');
end

Simulink.importExternalCTypes(which('kortex_wrapper_data.h'), 'OutputDir', 'temp');
gen3Kinova = kortex();
gen3Kinova.nbrJointActuators = 6;

%%%%%%%%%%%%%%% MODIFY HERE (START) %%%%%%%%%%%%%%%
% Update the IP address of the Kinova robot and your credential:
gen3Kinova.ip_address = '192.168.0.110';
gen3Kinova.user = 'admin';
gen3Kinova.password = 'admin';
%%%%%%%%%%%%%%% MODIFY HERE (END) %%%%%%%%%%%%%%%

disp('Connection credential configured.');
disp('Waiting for the connection with the robot ...');

isOk = gen3Kinova.CreateRobotApisWrapper();
if isOk
   disp('You are connected to the robot!'); 
else
   error('Failed to establish a valid connection!'); 
end

%%%%%%%%%%%%%%% MODIFY HERE (START) %%%%%%%%%%%%%%%
% Update the OPCUA IP address and communication port of the Festo MPS:
uaClient = opcua('192.168.0.1', 4840);
%%%%%%%%%%%%%%% MODIFY HERE (END) %%%%%%%%%%%%%%%

connect(uaClient)

convEnd = opcuanode(3, '"workpieceAtConvEnd"', uaClient);
stackStat = opcuanode(3, '"workpieceAvailable"', uaClient);


%% Send to home
jointCmd = wrapTo360(angles1(1,:));
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

pause(5)
ungrasp(gen3Kinova)

%% Simple rule-based task planner

goto(gen3Kinova, angles1, vel1, acc1, timestamp1)
i = 0;
task = 'pick';

while i < 3
    switch task
        case 'pick'
           if readValue(uaClient, stackStat)
               goto(gen3Kinova, angles2, vel2, acc2, timestamp2)
               grasp(gen3Kinova)
               goto(gen3Kinova, angles3, vel3, acc3, timestamp3)
               goto(gen3Kinova, angles4, vel4, acc4, timestamp4)
               goto(gen3Kinova, angles5, vel5, acc5, timestamp5)
               goto(gen3Kinova, angles6, vel6, acc6, timestamp6)
               ungrasp(gen3Kinova)
               goto(gen3Kinova, angles7, vel7, acc7, timestamp7)
               task = 'unload';
           else
               pause(0.2)
           end
        case 'unload'
            flagconv = readValue(uaClient2, convEnd);
            if flagconv ~= 1
                goto(gen3Kinova, angles8, vel8, acc8, timestamp8)
                goto(gen3Kinova, angles9, vel9, acc9, timestamp9)
                goto(gen3Kinova, angles10, vel10, acc10, timestamp10)
                grasp(gen3Kinova)
                goto(gen3Kinova, angles11, vel11, acc11, timestamp11)
                goto(gen3Kinova, angles12, vel12, acc12, timestamp12)
                ungrasp(gen3Kinova)
                i = i + 1;
                gostart(gen3Kinova, angles2)
                task = 'pick';
            else
                gostart(gen3Kinova, angles2)
                task = 'pick';
            end
    end               
end

while i == 3
    if readValue(uaClient2, convEnd) == 0
        gostart(gen3Kinova, angles9)
        goto(gen3Kinova, angles9, vel9, acc9, timestamp9)
        goto(gen3Kinova, angles10, vel10, acc10, timestamp10)
        grasp(gen3Kinova)
        goto(gen3Kinova, angles11, vel11, acc11, timestamp11)
        goto(gen3Kinova, angles12, vel12, acc12, timestamp12)
        ungrasp(gen3Kinova)
        i = i+1;
    else
        pause(0.2)
    end
end

disp('All done!');

%% Destroy API and Disconnect

isOk = gen3Kinova.DestroyRobotApisWrapper();

clear all
close all
clc
