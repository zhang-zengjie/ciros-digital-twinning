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
weights = [1 1 1 1 1 1];
initGuess = robot.homeConfiguration;
rng(0)

% Use set points in joint space as a simple heuristic motion planner

disp('Generating waypoints ...');
config = [0,0,0,0,0,0;
    1.05063582412566,0.874697131183675,-1.93907043826416,1.07391474582537,1.41007746952699,1.28383163625322;
    1.10980927718073,1.21555445307052,-1.81385271034238,1.11232705621068,1.52098234724983,1.47024242506871;
    1.05063582412566,0.874697131183675,-1.93907043826416,1.07391474582537,1.41007746952699,1.28383163625322;
    2.79992810818810,0.755111427428949,-1.93914645517960,-0.375676096711351,1.15102271081614,1.73020212502110;
    2.87191024373727,1.09966487262480,-1.89391592210307,-0.272508289702909,1.42817950486014,1.61052013488483;
    3.12008866350361,0.196181808035041,-2.01153474387633,-0.0361389102311442,0.637240096269369,1.59987284555512;    
    3.12728786013085,0.590456903856050,-1.50671935237257,1.56845077499863e-05,-1.04440916074029,1.55649508672785;
    3.10972135296205,1.01034110513006,-1.27208887667683,1.51519114890988e-05,-0.859155566129234,1.53892870378606;
    3.11102620995488,1.06119238755100,-1.14814436587883,1.39096678974257e-05,-0.932248774697881,1.54023532705449;
    3.11102486107994,0.958964099675282,-1.11958397207296,1.34678663665627e-05,-1.06303746787368,1.54023535477146;
    5.41217179706144,0.777137292099424,-1.52371786568498,-6.86836494848123e-06,-0.840736612133895,0.78383163625322;
    6.28318530717958,0.874697031183675,-1.93907013826416,1.07391424582537,1.41007246952699,1.28381163625322;
    7.33382113130525,0.874697131183675,-1.93907043826416,1.07391474582537,1.41007746952699,1.28383163625322;
    9.41047317171434,0.590456901703402,-1.50671935705786,1.56741351773115e-05,-1.04440915045267,1.55649510245250;
    3.12008866350361+2*pi,0.196181808035041,-2.01153474387633,-0.0361389102311442,0.637240096269369,1.59987284555512;
    1.05063582412566+2*pi,0.874697131183675,-1.93907043826416,1.07391474582537,1.41007746952699,1.28383163625322;
    ];

num_points = size(config, 1) - 1;

Ndof = numel(initGuess);
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

tf = WayPoints(end, 1);

disp('Waypoints successfully generated!');

%% Run Simulation

disp('Initializing simulation ...');
config1 = config(1,:);
out = sim('kinovaGen3_IK');

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
    if i >= num_points-3
        angles(:,1) = angles(:,1) - 360.0;
    end
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

%% Connect to robot

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

disp('Process starts!');
gohome(gen3Kinova, 3)

%% Pick and place
t_start = tic;
real_angles = goto(gen3Kinova, angles1, vel1, acc1, timestamp1, t_start, []);
real_angles = goto(gen3Kinova, angles2, vel2, acc2, timestamp2, t_start, real_angles);


%% Simple rule-based task planner
t_start = tic;
real_angles = goto(gen3Kinova, angles1, vel1, acc1, timestamp1, t_start, []);
mps_events = []; %#ok<*AGROW>
i = 0;
task = 'pick';
num_wkpc = 3;

while i < (num_wkpc - 1)
    switch task
        case 'pick'
            temp = toc(t_start);
            mps_events = [mps_events; [temp, 1]];
            
            if readValue(uaClient, stackStat)
               temp = toc(t_start);
               mps_events = [mps_events; [temp, 11]];
               
               real_angles = goto(gen3Kinova, angles2, vel2, acc2, timestamp2, t_start, real_angles);
               grasp(gen3Kinova)
               
               real_angles = goto(gen3Kinova, angles3, vel3, acc3, timestamp3, t_start, real_angles);
               real_angles = goto(gen3Kinova, angles4, vel4, acc4, timestamp4, t_start, real_angles);
               real_angles = goto(gen3Kinova, angles5, vel5, acc5, timestamp5, t_start, real_angles);
               ungrasp(gen3Kinova)
               
               real_angles = goto(gen3Kinova, angles6, vel6, acc6, timestamp6, t_start, real_angles);
               
               task = 'unload';
            else
               temp = toc(t_start);
               mps_events = [mps_events; [temp, 12]];
               pause(0.2)
           end
        case 'unload'
            temp = toc(t_start);
            mps_events = [mps_events; [temp, 2]];
            
            if readValue(uaClient, convEnd) == 0
                temp = toc(t_start);
                mps_events = [mps_events; [temp, 21]];
                
                real_angles = goto(gen3Kinova, angles7, vel7, acc7, timestamp7, t_start, real_angles);
                real_angles = goto(gen3Kinova, angles8, vel8, acc8, timestamp8, t_start, real_angles);
                real_angles = goto(gen3Kinova, angles9, vel9, acc9, timestamp9, t_start, real_angles);
                grasp(gen3Kinova)
                
                real_angles = goto(gen3Kinova, angles10, vel10, acc10, timestamp10, t_start, real_angles);
                real_angles = goto(gen3Kinova, angles11, vel11, acc11, timestamp11, t_start, real_angles);
                ungrasp(gen3Kinova)
                
                real_angles = goto(gen3Kinova, angles12, vel12, acc12, timestamp12, t_start, real_angles);
                real_angles = goto(gen3Kinova, angles13, vel13, acc13, timestamp13, t_start, real_angles);
                
                i = i + 1;
                task = 'pick';
            else
                temp = toc(t_start);
                mps_events = [mps_events; [temp, 22]];
                
                real_angles = goto(gen3Kinova, angles16, vel16, acc16, timestamp16, t_start, real_angles);
                task = 'pick';
            end
    end               
end

while i < num_wkpc
    temp = toc(t_start);
    mps_events = [mps_events; [temp, 3]];
    
    if readValue(uaClient, convEnd) == 0
        temp = toc(t_start);
        mps_events = [mps_events; [temp, 31]];
        
        real_angles = goto(gen3Kinova, angles14, vel14, acc14, timestamp14, t_start, real_angles);
        real_angles = goto(gen3Kinova, angles8, vel8, acc8, timestamp8, t_start, real_angles);
        real_angles = goto(gen3Kinova, angles9, vel9, acc9, timestamp9, t_start, real_angles);
        grasp(gen3Kinova)

        real_angles = goto(gen3Kinova, angles10, vel10, acc10, timestamp10, t_start, real_angles);
        real_angles = goto(gen3Kinova, angles11, vel11, acc11, timestamp11, t_start, real_angles);
        ungrasp(gen3Kinova)

        real_angles = goto(gen3Kinova, angles12, vel12, acc12, timestamp12, t_start, real_angles);
        real_angles = goto(gen3Kinova, angles13, vel13, acc13, timestamp13, t_start, real_angles);
        
        i = i + 1;
    else
        temp = toc(t_start);
        mps_events = [mps_events; [temp, 32]];
        pause(0.2)
    end
end

temp = toc(t_start);
mps_events = [mps_events; [temp, 4]];
clear t_start
disp('All done!');

%% Destroy API and disconnect

isOk = gen3Kinova.DestroyRobotApisWrapper();

clear
close all
clc