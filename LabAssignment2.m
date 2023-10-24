classdef LabAssignment2 < handle
    %LABASSIGNMENT2 Summary of this class goes here
    %   Detailed explanation goes here
    % Check
    
    properties
        panda;
        linearUR5;
        objPlates;
        plateCounter = 1;
        plateStackerCounter = 1;
        pandaJointAngles;
        pandaEnd;
        pandaGriperOffset = 0.05; % RADIUS OF A PLATE
        ur5JointAngles;
        ur5End;
        ur5GriperOffset = 0.06;
        pandaState;
        ur5State;
        plates;
        plateModel;
        plateStackerModel;
        plateJoints = [0,0];
        gui
    end
    
    methods
        function self = LabAssignment2()
            cla;
            % clf;
            clc;

            hold on
            self.InitialiseRobots();
            self.InitialiseEnvironment();
            self.InitialisePlates();
            % self.StartGui();
            self.RunRobot();            
        end
        
        function InitialiseRobots(self)
            self.panda = Panda(transl(1.6, 3.0, 0.95));

            %%  below change to the new linear ur5 (with gripper attachment)
            self.linearUR5 = LinearUR5custom(transl(0.5, 2.6, 0.95));
         
            self.UpdateRobots();
        end

        function StartGui(self)
            self.gui = GUI(self);
            qMatrix = self.pandaJointAngles;
            tic;
            while(1)
                self.gui.UpdateGUI();
                disp(['timestep ', num2str(toc)]);
                if self.gui.jog
                    tr = self.gui.StepRobot()
                    qMatrix = self.panda.model.ikcon(tr)
                    self.gui.UpdateSliders(qMatrix)
                    self.panda.model.animate(qMatrix);
                    self.gui.jog = false;
                else
                    qMatrix = self.gui.GetLinkValues();
                    self.panda.model.animate(qMatrix);
                end
                self.UpdateRobots();
                pause(0.5);
            end
        end

        function InitialiseEnvironment(self)
            Environment(); % Build the workspace for the robot
        end

        function InitialisePlates(self)
            self.objPlates = InitialisePlates(); % plot the plates in the workspace
            % self.objPlates.placeStacker

            plateNum = 1;
            for i = 1:3
                for j = 1:3
                    baseTr = transl(self.objPlates.plateLocations{plateNum}(1:3));
                    switch i
                        case 1
                            colour = 'red';
                        case 2
                            colour = 'blue';
                        case 3
                            colour = 'green';
                    end
                    self.plates{plateNum} = Plate(baseTr, colour);
                    plateNum = plateNum + 1;
                end
            end
        end




         function InitialisePlateStacker(self)
            self.objPlateStackers = InitialisePlateStacker(); % plot the plates in the workspace

            plateStackerNum = 1;
            for i = 1:3
                for j = 1:3
                    baseTr = self.objPlateStackers.initialTargetTransforms{plateStackerNum};
                    switch i
                        case 1
                            colour = 'red';
                        case 2
                            colour = 'blue';
                        case 3
                            colour = 'green';
                    end
                    self.plates{plateStackerNum} = Plate(baseTr, colour);
                    plateStackerNum = plateStackerNum + 1;
                end
            end
         end


        function RunRobot(self)                      
            % input('Press enter to begin')
            self.UpdateRobots();
        
            % Initialise log variable
            logData = struct('Time', [], 'Status', [], 'Transform', []); % Create the logData Structure
            
            % Log the start time and status
            logData.Time = {};
            logData.Time{end+1} = datestr(datetime('now'), 'yyyy-mm-dd HH:MM:SS'); % Log the time that the robot starts
            logData.Status{end+1} = 'Task Started';
            logData.Transform{end+1} = 'N/A';

            stack = 1;

            for i = 1:self.objPlates.numOfPlates
                disp(['Panda unstacking plate ', num2str(i)])
                self.pandaState = 1;
                self.ur5State = 1;
                self.plateModel = self.plates{self.plateCounter};
                % self.movePanda();
                for j = 1:6
                    % STATE 1 is panda moving to safe position above initial plate position (WITHOUT plate)
                    % STATE 2 is panda picking up plate from initial plate position
                    % STATE 3 is panda moving to safe position above initial plate position (WITH plate)
                    % STATE 4 is panda moving to safe position above final plate position (WITH plate)
                    % STATE 5 is panda placing plate in its final position
                    % STATE 6 is panda moving to safe position above final plate position (WITHOUT plate)

                    % Log the start time and status
                    logData.Time = {};
                    logData.Time{end+1} = datestr(datetime('now'), 'yyyy-mm-dd HH:MM:SS'); % Log the time that the robot starts
                    logData.Status{end+1} = ['Panda picking/placing plate ', num2str(i), ' in State ', num2str(j)];
                    logData.Transform{end+1} = mat2str(self.objPlates.initialTargetTransforms{i}(1:3, 4).');

                    self.MoveToPos();

                    % if (i == 4 && self.pandaState == 1) || (i == 7 && self.pandaState == 1)
                    %     self.MoveUR5(stack)
                    %     stack = stack + 3;
                    % end

                    self.pandaState = self.pandaState + 1;
                end
                self.plateCounter = self.plateCounter + 1;
            end
        
            self.MoveUR5(stack);

            % Display a message indicating the end of the task
            disp('Task Completed Successfully');
            % Log the end time and status
            logData.Time{end+1} = datestr(datetime('now'), 'yyyy-mm-dd HH:MM:SS');
            logData.Status{end+1} = 'Task Completed';
            logData.Transform{end+1} = 'N/A';
            % Save the log data
            save('logData.mat', 'logData');
        end

        function MoveToPos(self)
            self.UpdateRobots();
            steps = 10;
            switch self.pandaState
                case 1
                    disp('CASE 1')
                    targetPos = self.objPlates.safeInitialTargetTransforms{self.plateCounter};
                    rpy = rpy2tr(0, 180, 0, 'deg');
                case 2
                    disp('CASE 2')
                    targetPos = self.objPlates.initialTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 180, 0, 'deg');
                case 3
                    disp('CASE 3')
                    targetPos = self.objPlates.safeInitialTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 180, 0, 'deg');
                case 4
                    disp('CASE 4')
                    targetPos = self.objPlates.safeStackTargetTransforms{self.plateCounter};
                    rpy = rpy2tr(0, 180, 0, 'deg');
                case 5
                    disp('CASE 5')
                    targetPos = self.objPlates.stackTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 90, 180, 'deg');
                case 6
                    disp('CASE 6')
                    targetPos = self.objPlates.safeStackTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 90, 180, 'deg');
            end

            robotXYZ = self.pandaEnd.T; 
            robotXYZ = robotXYZ(1:3,4)';
            targetXYZ = targetPos(1:3,4)';

            if self.pandaState == 2 || self.pandaState == 3 || self.pandaState == 5 || self.pandaState == 6
                for i = 1:length(targetXYZ)
                    cartesianPath(:, i) = linspace(robotXYZ(i), targetXYZ(i), steps);
                end
                for i = 1:length(cartesianPath)
                    t = transl(cartesianPath(i,:)) * rpy;
                    pandaAngles = self.panda.model.ikcon(t, self.pandaJointAngles);
                    self.panda.model.animate(pandaAngles);
                    self.UpdateRobots();
                    if self.pandaState == 3 || self.pandaState == 4 || self.pandaState == 5
                        self.MovePlates();
                    end
                    drawnow();
                    % pause(0.1);
                end

            else
                qFinal = self.panda.model.ikcon(targetPos * rpy, self.pandaJointAngles);
                qMatrix = jtraj(self.pandaJointAngles, qFinal, steps);
                self.AnimatePanda(qMatrix);
            end
        end

        function AnimatePanda(self, qMatrix)
            for i = 1:length(qMatrix)
                self.panda.model.animate(qMatrix(i,:));
                self.UpdateRobots();
                if self.pandaState == 3 || self.pandaState == 4 || self.pandaState == 5
                    self.MovePlates();
                end
                drawnow();
                % pause(0.1);
            end
        end

        % function path = generate_arc_path(self, start_xyz, target_xyz, base_xyz, steps)
        %     % Input check
        %     start_xyz
        %     target_xyz
        %     base_xyz
        % 
        %     % Initialize path
        %     path = zeros(steps, length(start_xyz));
        % 
        %     % Check the state of 'panda'
        %     if self.pandaState == 2 || self.pandaState == 3 || self.pandaState == 5 || self.pandaState == 6
        %         for i = 1:length(target_xyz)
        %             path(:, i) = linspace(start_xyz(i), target_xyz(i), steps);
        %         end
        %     else
        %         % Calculate the midpoint of the line connecting start and target
        %         midpoint = (start_xyz + target_xyz) / 2;
        % 
        %         % Determine the direction from base to the midpoint in the XY plane
        %         direction_xy = [midpoint(1) - base_xyz(1), midpoint(2) - base_xyz(2)];
        %         direction_xy = direction_xy / norm(direction_xy);
        % 
        %         % Find the crest point 0.5m away from the base in the XY plane direction and midpoint Z height
        %         crest_xyz = [base_xyz(1:2) + 0.5 * direction_xy, midpoint(3)]
        % 
        %         % Linearly interpolate from start to crest and from crest to target
        %         first_half = zeros(ceil(steps/2), 3);
        %         second_half = zeros(floor(steps/2), 3);
        %         for i = 1:length(start_xyz)
        %             first_half(:, i) = linspace(start_xyz(i), crest_xyz(i), ceil(steps/2));
        %             second_half(:, i) = linspace(crest_xyz(i), target_xyz(i), floor(steps/2));
        %         end
        % 
        %         path = [first_half; second_half(2:end, :)];  % Concatenate to avoid repeating the crest value
        %     end
        % end

        function MoveUR5(self, stack)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % insert the platestacker robot placement once the plate stack
            % has been made 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            targetTransforms = cell(7);
            targetTransforms{1} = self.objPlates.stackTargetTransforms{stack} * rpy2tr(0, 90, 0, 'deg');
            targetTransforms{1}(1,4) = targetTransforms{1}(1,4) - 0.2;
            targetTransforms{2} = targetTransforms{1};
            targetTransforms{4} = targetTransforms{1};
            targetTransforms{1}(1,4) = targetTransforms{1}(1,4) - 0.1;
            targetTransforms{3} = targetTransforms{1};
            targetTransforms{4}(3,4) = targetTransforms{4}(3,4) + 0.45;
            targetTransforms{5} = self.objPlates.finalTargetTransforms{stack} * rpy2tr(0, 90, -90, 'deg');
            targetTransforms{6} = targetTransforms{5};
            targetTransforms{5}(2,4) = targetTransforms{5}(2,4) + 0.1;
            targetTransforms{7} = targetTransforms{5};

            for i = 1:length(targetTransforms)
                angles = self.linearUR5.model.ikcon(targetTransforms{i}, self.ur5JointAngles);
                qMatrix = jtraj(self.ur5JointAngles, angles, 10);
                for j = 1:length(qMatrix)
                    q = qMatrix(j,:);
                    self.linearUR5.model.animate(q);
                    drawnow();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    
                    % self.UpdateRobots();
                    % self.MovePlateStacker(); % To move the plate Stacker with the UR5
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                    pause(0.1);
                
                end
                self.UpdateRobots();
            end
        end

        function UpdateRobots(self)
            self.pandaJointAngles = self.panda.model.getpos();
            self.pandaEnd = self.panda.model.fkine(self.pandaJointAngles);
            self.ur5JointAngles = self.linearUR5.model.getpos();
            self.ur5End = self.linearUR5.model.fkine(self.ur5JointAngles);
        end

        function MovePlates(self)
            platePos = self.pandaEnd.T;
        
            % Extract the current z-direction of the end effector
            zDirection = platePos(1:3, 3);
            
            % Compute the offset in the global frame
            globalOffset = zDirection * (self.pandaGriperOffset);
            
            % Apply the offset to the current position
            platePos(1:3, 4) = platePos(1:3, 4) + globalOffset;
        
            if self.pandaState == 5
                self.plateModel.model.base = platePos * troty(-pi/2);
            else
                self.plateModel.model.base = platePos * trotx(-pi/2);
            end
            animate(self.plateModel.model, self.plateJoints);
        end
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function MovePlateStacker(self)
            plateStackerPos = self.ur5End.T;
        
            % Extract the current z-direction of the end effector
            zDirection = plateStackerPos(1:3, 3);
            
            % Compute the offset in the global frame
            globalOffset = zDirection * (self.LinearUR5Offset);
            
            % Apply the offset to the current position
            plateStackerPos(1:3, 4) = plateStackerPos(1:3, 4) + globalOffset;
        
            if self.linearUR5 == 1
                self.plateStackerModel.model.base = plateStackerPos * troty(-pi/2);
            else
                self.plateStackerModel.model.base = plateStackerPos * trotx(-pi/2);
            end
            animate(self.plateStackerModel.model, self.plateStackerJoints);
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end



