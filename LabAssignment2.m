classdef LabAssignment2 < handle
    %LABASSIGNMENT2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        yumi;
        linearUR5;
        objPlates;
        plateCounter = 1;
        yumiJointAngles;
        yumiEnd;
        yumiGriperOffset = 0.2;
        ur5JointAngles;
        ur5End;
        ur5GriperOffset = 0.06;
        yumiState;
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
            self.initialiseRobots();
            self.initialiseEnvironment();
            self.initialisePlates();
            self.runRobot();            
        end
        
        function initialiseRobots(self)
            self.yumi = Yumi;

            %%  below change to the new linear ur5 (with gripper attachment
            self.linearUR5 = LinearUR5;
        end

        function initialiseEnvironment(self)
            Environment(); % Build the workspace for the robot
        end

        function initialisePlates(self)
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

        function runRobot(self)                      
            input('Press enter to begin')
            self.updateRobots();
        
            % Initialise log variable
            logData = struct('Time', [], 'Status', [], 'Transform', []); % Create the logData Structure
            
            % Log the start time and status
            logData.Time = {};
            logData.Time{end+1} = datestr(datetime('now'), 'yyyy-mm-dd HH:MM:SS'); % Log the time that the robot starts
            logData.Status{end+1} = 'Task Started';
            logData.Transform{end+1} = 'N/A';

            for i = 1:self.objPlates.numOfPlates
                disp(['YuMi unstacking plate ', num2str(i)])
                self.yumiState = 1;
                self.plateModel = self.plates{self.plateCounter};
                for j = 1:6
                    % STATE 1 is Yumi moving to safe position above initial plate position (WITHOUT plate)
                    % STATE 2 is Yumi picking up plate from initial plate position
                    % STATE 3 is Yumi moving to safe position above initial plate position (WITH plate)
                    % STATE 4 is Yumi moving to safe position above final plate position (WITH plate)
                    % STATE 5 is Yumi placing plate in its final position
                    % STATE 6 is Yumi moving to safe position above final plate position (WITHOUT plate)
                    self.moveToPos();

                    self.yumiState = self.yumiState + 1;
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

        function moveToPos(self)
            self.updateRobots();
            steps = 10;
            switch self.yumiState
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
                    targetPos = self.objPlates.safeFinalTargetTransforms{self.plateCounter};
                    rpy = rpy2tr(0, 180, 0, 'deg');
                case 5
                    disp('CASE 5')
                    targetPos = self.objPlates.finalTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 90, 0, 'deg');
                case 6
                    disp('CASE 6')
                    targetPos = self.objPlates.safeFinalTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 180, 0, 'deg');
            end

            targetPos(3,4) = targetPos(3,4) + self.yumiGriperOffset;

            robotXYZ = self.yumiEnd.T; 
            robotXYZ = robotXYZ(1:3,4)';
            targetXYZ = targetPos(1:3,4)';
            
            cartesianPath = self.calculateMidpoints(robotXYZ, targetXYZ, steps);

            for i = 1:length(cartesianPath)
                t = transl(cartesianPath(i,:)) * rpy;
                yumiAngles = self.yumi.model.ikcon(t, self.yumiJointAngles);
                self.yumi.model.animate(yumiAngles);
                self.updateRobots();
                if self.yumiState == 3 || self.yumiState == 4 || self.yumiState == 5
                    self.movePlates();
                end
                drawnow();
                % pause(0.1);
            end

            realXYZ = self.yumi.model.fkine(yumiAngles).T;
            realXYZ = realXYZ(1:3,4)';

            disp(['distance from target = ', num2str(norm(realXYZ - targetXYZ)*1000), 'mm'])
        end

        function cartesianPath = calculateMidpoints(self, robotXYZ, targetXYZ, steps)
            initialXYZ = robotXYZ; % Get the XYZ coordinates from the robots end effector position
            finalXYZ = targetXYZ; % Get the XYZ coordinates from the final position transformation matrix

            if self.yumiState == 2 || self.yumiState == 3 || self.yumiState == 5 || self.yumiState == 6
                % Interpolate between robotXYZ and targetXYZ
                cartesianPath = zeros(steps, length(initialXYZ));
                % Generate the path
                for i = 1:length(initialXYZ)
                    cartesianPath(:, i) = linspace(initialXYZ(i), finalXYZ(i), steps);
                end
            else
                cartesianPath = zeros(steps, 3);
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
                    cartesianPath(i, :) = bezier_t; % Assign the calculated position to the Cartesian path matrix
                end
            end
        end

        function updateRobots(self)
            self.yumiJointAngles = self.yumi.model.getpos();
            self.yumiEnd = self.yumi.model.fkine(self.yumiJointAngles);
            self.ur5JointAngles = self.linearUR5.model.getpos();
            self.ur5End = self.linearUR5.model.fkine(self.ur5JointAngles);
        end

        function movePlates(self)
            platePos = self.yumiEnd.T;
            platePos(3,4) = platePos(3,4) - self.yumiGriperOffset;
            if self.yumiState == 5
                platePos(3,4) = platePos(3,4) - 0.05;
                self.plateModel.model.base = platePos * troty(-pi/2);
            else
                self.plateModel.model.base = platePos * trotx(-pi/2);
            end
            animate(self.plateModel.model, self.plateJoints)
        end
    end
end

