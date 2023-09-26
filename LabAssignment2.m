classdef LabAssignment2 < handle
    %LABASSIGNMENT2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        yumi;
        linearUR5;

    end
    
    methods
        function self = LabAssignment2()
            cla;
            % clf;
            clc;

            hold on
            self.initialiseRobots();
            self.initialiseEnvironment();
            % self.runRobot();            

        end
        
        function initialiseRobots(self)
            self.yumi = Yumi;
            self.linearUR5 = LinearUR5;
        end

        function initialiseEnvironment(self)
            Environment(); % Build the workspace for the robot
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

            %% ROBOT CODE GOES HERE %%
        
            % Display a message indicating the end of the task
            disp('Task Completed Successfully');
            % Log the end time and status
            logData.Time{end+1} = datestr(datetime('now'), 'yyyy-mm-dd HH:MM:SS');
            logData.Status{end+1} = 'Task Completed';
            logData.Transform{end+1} = 'N/A';
            % Save the log data
            save('logData.mat', 'logData');

        end
    end
end

