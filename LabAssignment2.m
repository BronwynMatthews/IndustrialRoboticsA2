classdef LabAssignment2 < handle
    properties
        panda;
        linearUR5;
        gripper1;
        gripper2;
        objPlates;
        plateCounter = 1;
        plateStackerCounter = 1;
        pandaJointAngles;
        pandaEnd;
        pandaGripperOffset = 0.12; % RADIUS OF A PLATE
        ur5JointAngles;
        ur5End;
        ur5GripperOffset = 0.25;
        pandaState;
        ur5State;
        plates;
        plateModel;
        plateStackerModel;
        plateJoints = [0,0];
        plateStackerJoints = [0,0];
        guiObj;
        stack;
        startup = true;
    end

    methods
        function self = LabAssignment2()
            cla;
            % clf;
            clc;

            h = findall(0, 'Type', 'figure', 'Name', 'MATLAB App');
            close(h);

            hold on

            self.InitialiseRobots();
            self.guiObj = GUI(self.linearUR5, self.panda);
            self.InitialiseEnvironment();
            self.InitialisePlates();
            self.RunRobot();
        end

        function InitialiseRobots(self)
            self.panda = Panda(transl(1.6, 3.0, 0.95));

            %%  below change to the new linear ur5 (with gripper attachment)
            self.linearUR5 = LinearUR5custom(transl(0.4, 2.6, 0.95));

            self.UpdateRobots();

            self.gripper1 = PandaGripper(self.pandaEnd, 1);
            self.gripper2 = PandaGripper(self.pandaEnd, 2);
        end

        function CheckGUI(self)
            self.guiObj.UpdateGUI;
            while ~self.guiObj.running
                if ~self.guiObj.estop
                    [tempQMatrix, robot] = self.guiObj.GetSliderValues();
                    robot.model.animate(tempQMatrix);
                end
                pause(0.1);
                self.guiObj.UpdateGUI;
            end           
        end

        function InitialiseEnvironment(self)
           Environment(); % Build the workspace for the robot

            % Create an instance of the Rectangles class
            rectangles = Rectangles();
            rectangles.draw(gca);
            % lightCurtains = LightCurtains();
            % lightCurtains.draw(gca);
            
            % Define parameters for the rectangular prism
            % lower = [-2.5, 3.3, 0];
            % upper = [4, 3.4, 3];
            % plotOptions.plotVerts = true;
            % plotOptions.plotEdges = true;
            % plotOptions.plotFaces = true;
            % 
            % % Call the RectangularPrism method
            % [vertex, face, faceNormals] = rectangles.RectangularPrism(lower, upper, plotOptions, gca);
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

        function RunRobot(self)
            self.UpdateRobots();
            stackCounter = 1;
            % Initialise log variable
            logData = struct('Time', [], 'Status', [], 'Transform', []); % Create the logData Structure

            % Log the start time and status
            logData.Time = {};
            logData.Time{end+1} = datestr(datetime('now'), 'yyyy-mm-dd HH:MM:SS'); % Log the time that the robot starts
            logData.Status{end+1} = 'Task Started';
            logData.Transform{end+1} = 'N/A';

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

                    if (i == 4 && self.pandaState == 1) || (i == 7 && self.pandaState == 1)
                        if i == 4
                            colour = 'red';
                        elseif i == 7
                            colour = 'blue';
                        end

                        try
                            delete(self.objPlates.stackers{stackCounter})
                            delete(self.plates{i-3});
                            delete(self.plates{i-2});
                            delete(self.plates{i-1});
                        end
                        pos = self.objPlates.plateStack{i - 3}(1:3);
                        pos(3) = pos(3) - 0.04;
                        self.stack{stackCounter} = PlateStacker(transl(pos), colour);
                        self.MoveUR5(stackCounter)
                        stackCounter = stackCounter + 1;
                    end

                    self.pandaState = self.pandaState + 1;
                end
                self.plateCounter = self.plateCounter + 1;
            end

            colour = 'green';
            try
                delete(self.objPlates.stackers{3})
                delete(self.plates{7});
                delete(self.plates{8});
                delete(self.plates{9});
            end
            pos = self.objPlates.plateStack{7}(1:3);
            pos(3) = pos(3) - 0.04;
            self.stack{3} = PlateStacker(transl(pos), colour);

            self.MoveUR5(stackCounter);

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
                    rpy = rpy2tr(-90, 180, 90, 'deg');
                case 5
                    disp('CASE 5')
                    targetPos = self.objPlates.stackTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(-90, 180, 90, 'deg');
                case 6
                    disp('CASE 6')
                    targetPos = self.objPlates.safeStackTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(-90, 180, 90, 'deg');
            end
            
            targetPos = targetPos * rpy;

            if self.pandaState == 1 || self.pandaState == 4 || self.pandaState == 6 
                qFinal = self.panda.model.ikcon(targetPos, self.pandaJointAngles);
                qMatrix = jtraj(self.pandaJointAngles, qFinal, steps);
                self.AnimatePanda(qMatrix);
                
            else
                qMatrix = ResolvedMotionRateControl(self.panda, targetPos, 'vertical');
                self.AnimatePanda(qMatrix);
            end
        end

        function AnimatePanda(self, qMatrix)
            for i = 1:length(qMatrix)
                self.panda.model.animate(qMatrix(i,:));
                self.UpdateRobots();
                if self.pandaState == 3 || self.pandaState == 4 || self.pandaState == 5
                    self.MovePlates();
                    self.AnimateGripper('close')
                else
                    self.AnimateGripper('open')
                end
                drawnow();
                % pause(0.1);
            end
        end

        function MoveUR5(self, stack)
            self.plateStackerModel = self.stack{stack};

            targetTransforms = cell(7);
            targetTransforms{1} = self.plateStackerModel.model.base.T;
            if stack == 1
                targetTransforms{1}(1,4) = targetTransforms{1}(1,4) - self.ur5GripperOffset - 0.15;
                targetTransforms{1} = targetTransforms{1} * rpy2tr(0, -90, 90, 'deg');  
                
                targetTransforms{2} = targetTransforms{1};
                targetTransforms{2}(1,4) = targetTransforms{2}(1,4) + 0.15;

                targetTransforms{3} = targetTransforms{2};
                targetTransforms{3} = targetTransforms{2} * rpy2tr(-90, 0, 0, 'deg');
            else
                targetTransforms{1}(2,4) = targetTransforms{1}(2,4) + self.ur5GripperOffset + 0.15;
                targetTransforms{1} = targetTransforms{1} * rpy2tr(0, -90, 0, 'deg');  

                targetTransforms{2} = targetTransforms{1};
                targetTransforms{2}(2,4) = targetTransforms{2}(2,4) - 0.15;

                targetTransforms{3} = targetTransforms{2};
            end
            targetTransforms{3}(3,4) = targetTransforms{3}(3,4) + 0.45;

            targetTransforms{4} = self.objPlates.finalTargetTransforms{stack} * rpy2tr(0, -90, 90, 'deg');
            targetTransforms{4}(2,4) = targetTransforms{4}(2,4) + self.ur5GripperOffset + 0.25;

            targetTransforms{5} = targetTransforms{4};
            targetTransforms{5}(2,4) = targetTransforms{5}(2,4) - 0.25;

            targetTransforms{6} = targetTransforms{4};

            targetTransforms{7} = self.linearUR5.model.fkine(self.linearUR5.q0);

            for i = 1:length(targetTransforms)
                T1 = targetTransforms{i};

                if i == 4 % using RMRC
                    qMatrix = ResolvedMotionRateControl(self.linearUR5, T1, 'horizontal');
                else
                    angles = self.linearUR5.model.ikcon(T1, self.ur5JointAngles);
                    qMatrix = jtraj(self.ur5JointAngles, angles, 10);
                end

                for j = 1:size(qMatrix,1)
                    q = qMatrix(j,:);
                    self.linearUR5.model.animate(q);
                    if i >= 3 && i < 6
                        self.MovePlateStacker()
                        drawnow();
                    end
                    drawnow();
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

            if ~self.startup()
                self.CheckGUI();
                % check collision
                % check lightcurtain
            end

            self.startup = false;           
        end

        function MovePlates(self)
            self.UpdateRobots();
            platePos = self.pandaEnd.T;

            % Extract the current z-direction of the end effector
            zDirection = platePos(1:3, 3);

            % Compute the offset in the global frame
            globalOffset = zDirection * (self.pandaGripperOffset);

            % Apply the offset to the current position
            platePos(1:3, 4) = platePos(1:3, 4) + globalOffset;

            self.plateModel.model.base = platePos * trotx(-pi/2);
            animate(self.plateModel.model, self.plateJoints);
        end

        % Below is the function to check collisions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % function CheckCollisions(self)
        %
        %
        %     for objectCoordinate <
        %         % Stop robot movement
        %         disp("Robot collision occured!! Oh No!!")
        %     end
        %
        % end


        function MovePlateStacker(self)
            self.UpdateRobots();
            plateStackerPos = self.ur5End.T;

            % Extract the current z-direction of the end effector
            zDirection = plateStackerPos(1:3, 3);

            % Compute the offset in the global frame
            globalOffset = zDirection * self.ur5GripperOffset;

            % Apply the offset to the current position
            plateStackerPos(1:3, 4) = plateStackerPos(1:3, 4) + globalOffset;

            self.plateStackerModel.model.base = plateStackerPos * troty(pi/2);
            animate(self.plateStackerModel.model, self.plateStackerJoints);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        

        function AnimateGripper(self, state)
            self.gripper1.model.base = self.gripper1.model.base.T;
            self.gripper1.model.base = self.pandaEnd.T * trotx(pi);
            self.gripper2.model.base = self.gripper1.model.base;

            self.gripper1.model.base = self.gripper1.model.base.T * troty(-pi/2) * trotz(pi) * trotx(-pi/2);
            self.gripper2.model.base = self.gripper2.model.base.T * troty(pi/2) * trotx(-pi/2);
 
            if strcmp(state, 'open')
                self.gripper1.model.animate(0.03)
                self.gripper2.model.animate(0.03)
            else
                self.gripper1.model.animate(0.005)
                self.gripper2.model.animate(0.005)
            end

        end
    end
end



