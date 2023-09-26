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
        yumiGriperOffset = 0.06
        safeOffset = 0.3;
        ur5JointAngles;
        ur5End;
        ur5GriperOffset = 0.06
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
            self.linearUR5 = LinearUR5;
        end

        function initialiseEnvironment(self)
            Environment(); % Build the workspace for the robot
        end

        function initialisePlates(self)
            self.objPlates = Plates(); % plot the plates in the workspace
        end

        function runRobot(self)                      
            input('Press enter to begin')
        
            % Initialise log variable
            logData = struct('Time', [], 'Status', [], 'Transform', []); % Create the logData Structure
            
            % Log the start time and status
            logData.Time = {};
            logData.Time{end+1} = datestr(datetime('now'), 'yyyy-mm-dd HH:MM:SS'); % Log the time that the robot starts
            logData.Status{end+1} = 'Task Started';
            logData.Transform{end+1} = 'N/A';

            for i = 1:self.objPlates.numOfPlates
                self.updateRobots();
                self.moveToPos(self.objPlates.initialTargetTransforms{i})

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

        function moveToPos(self, targetPos)
            steps = 50;
            finalT = targetPos * rpy2tr(0, 180, 0, 'deg');
            finalT(3,4) = finalT(3,4) + self.yumiGriperOffset
            startAngles = self.yumiJointAngles;
            endAngles = self.yumi.model.ikcon(finalT, startAngles);
            qMatrix = jtraj(startAngles, endAngles, steps)
            for i = 1:length(qMatrix)
                self.yumi.model.animate(qMatrix(i,:));
                % pause(0.1);
            end
        end            

        function updateRobots(self)
            self.yumiJointAngles = self.yumi.model.getpos();
            self.yumiEnd = self.yumi.model.fkine(self.yumiJointAngles);
            self.ur5JointAngles = self.linearUR5.model.getpos();
            self.ur5End = self.linearUR5.model.fkine(self.ur5JointAngles);
        end
    end
end

