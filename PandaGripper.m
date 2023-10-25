classdef PandaGripper < RobotBaseClass
    properties(Access = public)              
        plyFileNameStem = 'PandaGripper'
        finger1
        finger2  
    end
    
    methods
        %% Define robot Function 
        function self = PandaGripper(baseTr)
            self.CreateModel();

            if nargin < 1            
                baseTr = eye(4);
                baseTr = baseofPanda;
            end

            self.finger1.base = self.finger1.base * baseTr * trotz(pi/2);
            self.finger2.base = self.finger2.base * baseTr * trotz(pi/2);

            self.PlotAndColourRobot();    
        end

        %% Create the plate model
        function CreateModel(self)   
            
            % Prismatic link 1
            L1 = Link('prismatic', 'a', 0, 'alpha', 0, 'theta', 0, 'qlim', [-0.05 0.05]);
            self.finger1 = SerialLink(L1, 'name', 'finger1');

            % Prismatic link 2
            L2 = Link('prismatic', 'a', 0, 'alpha', 0, 'theta', 0, 'qlim', [-0.05 0.05]);
            self.finger2 = SerialLink(L2, 'name', 'finger2');
        end

        %% in the main code set the fingerOpen status to true which means it will open others it will shut
        function OpenAndClose(fingerOpen)

            if fingerOpen == true
                gripper.finger1.plot(0.03); 
                gripper.finger2.plot(-0.03);
            else 
                gripper.finger1.plot(0.01); 
                gripper.finger2.plot(-0.01);
            end

        end
    end
end