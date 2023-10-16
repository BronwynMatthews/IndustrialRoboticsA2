classdef LabAssignment2 < handle
    %LABASSIGNMENT2 Summary of this class goes here
    %   Detailed explanation goes here
    % Check
    
    properties
        panda;
        linearUR5;
        objPlates;
        plateCounter = 1;
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
        plateJoints = [0,0];
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
            self.RunRobot();            
        end
        
        function InitialiseRobots(self)
            self.panda = Panda(transl(1.5, 2.6, 1.0));

            %%  below change to the new linear ur5 (with gripper attachment
            self.linearUR5 = LinearUR5custom(transl(0.5,2.6,1.0));
        end

        function InitialiseEnvironment(self)
            Environment(); % Build the workspace for the robot
        end

        function InitialisePlates(self)
            self.objPlates = InitialisePlates(); % plot the plates in the workspace

            plateNum = 1;
            for i = 1:3
                for j = 1:3
                    baseTr = self.objPlates.initialTargetTransforms{plateNum};
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
            input('Press enter to begin')
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

                    if (i == 3 && self.pandaState == 6) || (i == 6 && self.pandaState == 6) || (i == 9 && self.pandaState == 6)
                        self.MoveUR5(stack)
                        stack = stack + 3;
                    end

                    self.pandaState = self.pandaState + 1;
                end
                self.plateCounter = self.plateCounter + 1;
            end
        
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
                    rpy = rpy2tr(0, 90, 90, 'deg');
                case 6
                    disp('CASE 6')
                    targetPos = self.objPlates.safeStackTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 90, 90, 'deg');
            end

            robotXYZ = self.pandaEnd.T; 
            robotXYZ = robotXYZ(1:3,4)';
            targetXYZ = targetPos(1:3,4)';
            
            cartesianPath = self.CalculateMidpoints(robotXYZ, targetXYZ, steps);

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

            realXYZ = self.panda.model.fkine(pandaAngles).T;
            realXYZ = realXYZ(1:3,4)';

            disp(['distance from target = ', num2str(norm(realXYZ - targetXYZ)*1000), 'mm'])
        end

        function CartesianPath = CalculateMidpoints(self, robotXYZ, targetXYZ, steps)
            initialXYZ = robotXYZ; % Get the XYZ coordinates from the robots end effector position
            finalXYZ = targetXYZ; % Get the XYZ coordinates from the final position transformation matrix

            if self.pandaState == 2 || self.pandaState == 3 || self.pandaState == 5 || self.pandaState == 6
                % Interpolate between robotXYZ and targetXYZ
                CartesianPath = zeros(steps, length(initialXYZ));
                % Generate the path
                for i = 1:length(initialXYZ)
                    CartesianPath(:, i) = linspace(initialXYZ(i), finalXYZ(i), steps);
                end
            else
                CartesianPath = zeros(steps, 3);
                midpoint = (initialXYZ + finalXYZ) / 2; % Calculate the midpoint between the initial and final positions
                vec = finalXYZ - initialXYZ; % Calculate the vector from the initial to the final position
                offset_distance = 0.75 - 0.25 * abs(midpoint(2)) / 1;  % Calculate the offset distance based on the Y coordinate of the midpoint
                offset_distance = max(0.25, min(0.75, offset_distance));  % Ensure the offset distance is within a specific range
                rotatedVec = [-vec(2), vec(1), 0]; % Create a vector rotated by 90 degrees in the XY plane
                rotatedVec = offset_distance * rotatedVec / norm(rotatedVec); % Scale the rotated vector by the offset distance
                crest = midpoint - rotatedVec;
                % Generate the arc
                for i = 1:steps
                    t = (i - 1) / (steps - 1);  % Define a parameter t that linearly interpolates from 0 to 1
                    oneMinusT = 1 - t; % Calculate (1 - t) for use in the Bezier curve formula
                    % Use the formula for a quadratic Bezier curve to generate the arc
                    bezier_t = oneMinusT^2 * initialXYZ + 2 * oneMinusT * t * crest + t^2 * finalXYZ; % Calculate the position on the Bezier curve at parameter t
                    CartesianPath(i, :) = bezier_t; % Assign the calculated position to the Cartesian path matrix
                end
            end
        end

        function MoveUR5(self, stack)
            stack
            targetTransforms = cell(7);
            targetTransforms{1} = self.objPlates.stackTargetTransforms{stack} * rpy2tr(0, 90, 0, 'deg');
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
                    pause(0.1);
                end
                self.UpdateRobots();
            end
        end

        function SplitAnimation(self, qMatrixPanda, qMatrixUR5)
            for i = 1:length(qMatrixPanda)
                q = qMatrixPanda()
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
    end
end

