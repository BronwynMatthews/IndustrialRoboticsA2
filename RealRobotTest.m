%% Calculate joint positions
clc;
cla;

robot = UR3;
n = robot.model.n;


axis([-0.5 0.5 -0.5 0.6 0.0 0.6]);
hold on

q0 = [0 -90 0 -90 -90 0];
q0 = deg2rad(q0);

robot.model.animate(q0);

plateOffset = 0.1;

initialStack = {
    [0.3 0.2 0.1]; 
    [0.4 0.0 0.05];
    [0.4 -0.2 0.05];
};

trStack = cell(3,1);
for i = 1:length(trStack)
    trStack{i} = transl(initialStack{i});
end

finalStack = {
    [0.2, -0.4, 0.3];
    [0.0, -0.4, 0.3];
    [-0.2, -0.4, 0.3];
};

trFinal = cell(3,1);
for i = 1:length(trFinal)
    trFinal{i} = transl(finalStack{i});
end



targetTransforms = cell(21,1);
stack = 1;

% for i = 1:length(trStack)
%     targetTransforms{1} = trStack{1} * rpy2tr(90, -180, 90, 'deg');
%     % targetTransforms{1*i} = targetTransforms{1*i} * rpy2tr(90, -180,90, 'deg');
%     % targetTransforms{1*i}(1,4) = targetTransforms{1*i}(1,4) - plateOffset - 0.2;
%     % targetTransforms{1*i} = targetTransforms{1*i} * rpy2tr(90, -180, 90, 'deg');
% 
%     targetTransforms{2*i} = trStack{i} * rpy2tr(0, -90, 0, 'deg');
%     targetTransforms{2*i}(1,4) = targetTransforms{2*i}(1,4) - plateOffset;
% 
%     targetTransforms{3*i} = targetTransforms{2*i} * rpy2tr(0, 0, -90, 'deg');
%     targetTransforms{3*i}(3,4) = targetTransforms{3*i}(3,4) + 0.25;
% 
%     targetTransforms{4*i} = trFinal{stack} * rpy2tr(0, -90, 90, 'deg');
%     targetTransforms{4*i}(2,4) = targetTransforms{4*i}(2,4) + plateOffset + 0.2;
% 
%     targetTransforms{5*i} = trFinal{stack} * rpy2tr(0, -90, 90, 'deg');
%     targetTransforms{5*i}(2,4) = targetTransforms{5*i}(2,4) + plateOffset;
% 
%     targetTransforms{6*i} = targetTransforms{4*i};
% 
%     targetTransforms{7*i} = robot.model.fkine(q0);
% 
%     stack = stack + 1;
% 
%     % for i = 1:7
%     %     targetTransforms{i}
%     % end
%     % disp('new stack')
% end


tr = cell(729,1);
count = 1;
for i = -180:45:180
    for j = -180:45:180
        for k = -180:45:180
            tr{count} = trStack{1} * rpy2tr(k,j,i, 'deg');
            
            jointAngles = robot.model.ikcon(tr{count}, q0);
            xyz = robot.model.fkine(jointAngles);
            xyz = xyz.T;
            xyz = xyz(1:3,4)';
            dist = norm(xyz - initialStack{1});
            if dist <= 0.005
                robot.model.animate(jointAngles)

                drawnow();
                pause(1)

                disp(['count = ', num2str(count), ' r = ', num2str(k), ' p = ', num2str(j),' y = ', num2str(i)]);
            end 
            count = count + 1;
        end
    end
end


%%

steps = 10;

qMatrix = zeros(steps, n);

robot = UR3;

% for i = 1:length(targetTransforms)
for i = 1:1
    i
    tr = trStack{1} * rpy2tr(-135, 0, -90, 'deg');
    jointAngles = robot.model.ikcon(tr, q0);
    robot.model.teach(jointAngles)
    % qMatrix = jtraj(q0, jointAngles, 10);
    % qMatrix = ResolvedMotionRateControl(robot, tr)
    % for j = 1:length(qMatrix)
    %     robot.model.animate(qMatrix(j, :)); 
    %     drawnow();
    % 
    %     pause(0.5);
    % end
    % robot.model.animate(jointAngles)
    xyz = robot.model.fkine(robot.model.getpos());
    xyz = xyz.T;
    xyz = xyz(1:3,4)'
end



%% Subscribe to Robot 
rosinit('192.168.27.1'); % If unsure, please ask a tutor
%% SUBSCRIBE TO JOINT STATES
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
goal.Trajectory.JointNames = jointNames;

%% USE THE BELOW TO TEST INDIVIDUAL JOINT STATES
    % Query Current joint state
    jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
    pause(2); % Pause to give time for a message to appear
    currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
    currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
    
    
    % Variable for Joint States
    jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
    [client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
    goal.Trajectory.JointNames = jointNames;
    goal.Trajectory.Header.Seq = 1;
    goal.Trajectory.Header.Stamp = rostime('Now','system');
    goal.GoalTimeTolerance = rosduration(0.05);
    bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
    durationSeconds = 5; % This is how many seconds the movement will take
    
    startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    startJointSend.Positions = currentJointState_123456;
    startJointSend.TimeFromStart = rosduration(0);     
      
    endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    nextJointState_123456 = qMatrix(1,:); %% INPUT JOINT NUMBER YOU WISH TO TEST HERE
    endJointSend.Positions = nextJointState_123456;
    endJointSend.TimeFromStart = rosduration(durationSeconds);
    
    goal.Trajectory.Points = [startJointSend; endJointSend];
    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
    
    % Send Command
    % input('Press Enter to confirm sending joint angles to robot')
    
    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
    sendGoal(client,goal);

    %% RUN THE WHOLE ROBOT PROGRAM
for i = 1:length(initialStack)
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
    nextJointState_123456 = qMatrix(i,:);
    endJointSend.Positions = nextJointState_123456;
    endJointSend.TimeFromStart = rosduration(durationSeconds);
    
    goal.Trajectory.Points = [startJointSend; endJointSend];
    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

    % Send Command
    % input('Press Enter to confirm sending joint angles to robot')

    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
    sendGoal(client,goal);

    pause(6);
end
