classdef PandaGripper < RobotBaseClass
    properties(Access = public)              
        plyFileNameStem = 'PandaGripper'
        finger1
        finger2
        openGripper = 0.03
        closeGripper = 0.005
    end

    methods
        %% Define robot Function 
        function self = PandaGripper(robotEnd, gripperNo)
            if nargin < 1            
                robotEnd = transl(0,0,0);
                gripperNo = 2;
            end
            
            self.CreateModel();

            self.model.base = self.model.base.T;
            self.model.base = robotEnd.T * trotx(pi);

            % Align Gripper with robot end effector orientation
            if gripperNo == 1
                self.model.base = self.model.base.T * troty(-pi/2) * trotz(pi) * trotx(-pi/2);
            else
                self.model.base = self.model.base.T * troty(pi/2) * trotx(-pi/2);
            end

            % self.PlotAndColourRobot();  
            self.model.plot(self.closeGripper, 'nowrist', 'nobase', 'nojoints', 'noshadow'); 
        end

        %% Create the plate model
        function CreateModel(self)   

            link(1) = Link([0      0        0.03         0    1]); 
                       
            % Incorporate joint limits
            link(1).qlim = [0.005 0.03];

            self.model = SerialLink(link,'name',self.name);

            % % Prismatic link 1
            % L1 = Link('a', 0, 'alpha', 0, 'd', 0.005, 'theta', 0, 'prismatic');
            % L1.qlim = [-20 30]*pi/180;
            % self.finger1 = SerialLink(L1, 'name', 'finger1');
            % self.finger1.base = self.finger1.base * baseTr * trotz(pi/2);
            % 
            % % Prismatic link for finger2 using provided DH parameters
            % L2 = Link('a', 0, 'alpha', 0, 'd', 0.005, 'theta', 0.15, 'prismatic');
            % L2.qlim = [-180 180]*pi/180;
            % self.finger2 = SerialLink(L2, 'name', 'finger2');
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