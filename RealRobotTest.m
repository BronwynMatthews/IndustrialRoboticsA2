%% Calculate joint positions
clc;
cla;

robot = UR3;
n = robot.model.n;


axis([-0.5 0.5 -0.5 0.6 0.0 0.8]);
hold on

q0 = [0 -90 0 -90 0 0];
q0 = deg2rad(q0);

robot.model.animate(q0);

plateOffset = 0.1;

initialStack = {
    [0.3 -0.1 0.1]; 
    [0.4 -0.3 0.1];
    % [0.2 0.2 0.1];
};

trStack = cell(2,1);
for i = 1:length(trStack)
    trStack{i} = transl(initialStack{i});
end

finalStack = {
    [0.2, -0.5, 0.3];
    [0.0, -0.5, 0.3];
    % [-0.2, -0.5, 0.3];
};

trFinal = cell(2,1);
for i = 1:length(trFinal)
    trFinal{i} = transl(finalStack{i});
end

% for i = -180:90:180
%     for j = -180:90:180
%         for k = -180:90:180
% 
%             % xyz = robot.model.fkine(jointAngles);
%             % xyz = xyz.T;
%             % xyz = xyz(1:3,4)'
%             % point = initialStack{1}(:)'
%             % dist = norm(xyz, point);
%             % % if abs(dist) <= 0.005
%                 i
%                 j
%                 k
%                 robot.model.animate(jointAngles)
%                 drawnow();
%                 pause(1);
%             % end
% 
%         end
%     end
% end

%
robot.model.animate(q0);
drawnow();
pause(0.5);
tr = trStack{1} * rpy2tr(-90, 0, 180, 'deg');
tr(2,4) = tr(2,4) + plateOffset + 0.2;
jointAngles = robot.model.ikcon(tr, q0);
robot.model.animate(jointAngles)
drawnow();

                %%


targetTransforms = cell(13,1);


targetTransforms{1} = trStack{1} * rpy2tr(-90, 0, 180, 'deg');
targetTransforms{1}(2,4) = targetTransforms{1}(2,4) + plateOffset + 0.2;

targetTransforms{2} = trStack{1} * rpy2tr(-90, 0, 180, 'deg');
targetTransforms{2}(2,4) = targetTransforms{2}(2,4) + plateOffset;

targetTransforms{3} = targetTransforms{2};
targetTransforms{3}(3,4) = targetTransforms{3}(3,4) + 0.2;

targetTransforms{4} = trFinal{1} * rpy2tr(-90, 0, 180, 'deg');
targetTransforms{4}(2,4) = targetTransforms{4}(2,4) + plateOffset + 0.2;

targetTransforms{5} = trFinal{1} * rpy2tr(-90, 0, 180, 'deg');
targetTransforms{5}(2,4) = targetTransforms{5}(2,4) + plateOffset;

targetTransforms{6} = targetTransforms{4};

targetTransforms{7} = trStack{2} * rpy2tr(-90, 0, 180, 'deg');
targetTransforms{7}(2,4) = targetTransforms{7}(2,4) + plateOffset + 0.2;

targetTransforms{8} = trStack{2} * rpy2tr(-90, 0, 180, 'deg');
targetTransforms{8}(2,4) = targetTransforms{8}(2,4) + plateOffset;

targetTransforms{9} = targetTransforms{8};
targetTransforms{9}(3,4) = targetTransforms{9}(3,4) + 0.2;

targetTransforms{10} = targetTransforms{4};

targetTransforms{11} = trFinal{2} * rpy2tr(-90, 0, 180, 'deg');
targetTransforms{11}(2,4) = targetTransforms{11}(2,4) + plateOffset + 0.2;

targetTransforms{12} = trFinal{2} * rpy2tr(-90, 0, 180, 'deg');
targetTransforms{12}(2,4) = targetTransforms{12}(2,4) + plateOffset;

targetTransforms{13} = targetTransforms{11};

realJointStates = cell(14,1);
realJointStates{14} = q0;

robot.model.animate(q0);
drawnow();
pause(0.5)

for i = 1:13
    i
    tr = targetTransforms{i};
    jointAngles = robot.model.ikcon(tr, robot.model.getpos());
    realJointStates{i} = jointAngles;
    robot.model.animate(jointAngles)
    drawnow();
    pause(1);
end
robot.model.animate(q0);
drawnow();
pause(0.5)




%% Subscribe to Robot 
rosshutdown();
rosinit('192.168.27.1'); % If unsure, please ask a tutor
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
pause(2); % Pause to give time for a message to appear

jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;

goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take



%% USE THE BELOW TO TEST INDIVIDUAL JOINT STATES
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
for i = 1:7
    pause(1); % Pause to give time for a message to appear
    
    currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
    currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
    
    % Variable for Joint States
    
    goal.Trajectory.Header.Stamp = rostime('Now','system');

    startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    startJointSend.Positions = currentJointState_123456;
    startJointSend.TimeFromStart = rosduration(0);     
      
    endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    nextJointState_123456 = realJointStates{i};
    endJointSend.Positions = nextJointState_123456;
    endJointSend.TimeFromStart = rosduration(durationSeconds);
    
    goal.Trajectory.Points = [startJointSend; endJointSend];
    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
    
    % Send Command
    
    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
    disp('goal sent')
    sendGoal(client,goal);

    pos = (jointStateSubscriber.LatestMessage.Position)';

    while ~all(abs(pos(:) - realJointStates{i}(:)) < 0.05)
        disp('here')
        pause(1)
        pos = (jointStateSubscriber.LatestMessage.Position)'; % Update joint positions
    end

end

    %% RUN THE WHOLE ROBOT PROGRAM
for i = 1:7
    % Query Current joint staterosh
    
    pause(2); % Pause to give time for a message to appear
    currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
    currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
    
    % Variable for Joint States
    
    % goal.Trajectory.JointNames = jointNames;
    goal.Trajectory.Header.Seq = 1;
    goal.Trajectory.Header.Stamp = rostime('Now','system');
    goal.GoalTimeTolerance = rosduration(0.05);
    bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
    durationSeconds = 5; % This is how many seconds the movement will take
    
    startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    startJointSend.Positions = currentJointState_123456;
    startJointSend.TimeFromStart = rosduration(0);     
          
    endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    nextJointState_123456 = realJointStates{i};
    endJointSend.Positions = nextJointState_123456;
    endJointSend.TimeFromStart = rosduration(durationSeconds);
    
    goal.Trajectory.Points = [startJointSend; endJointSend];
    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

    % Send Command
    % input('Press Enter to confirm sending joint angles to robot')

    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
    disp('goal sent')
    sendGoal(client,goal);

    pause(7);
end
