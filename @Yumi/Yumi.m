classdef Yumi < RobotBaseClass
    properties(Access = public)   
        plyFileNameStem = 'abbyumi';
        q0 = [0,0,0,0,0,0,0]
    end
    
    methods
%% Constructor
        function self = Yumi(baseTr)
			self.CreateModel();
            if nargin < 1			
				% baseTr = transl(1.5, 2.6 ,1.0);
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr;
                        
            self.PlotAndColourRobot();
            % self.model.teach();  
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.166,'a',-0.03,'alpha',-pi/2,'qlim',deg2rad([-168.5 168.5]), 'offset',0);
            link(2) = Link('d',0,'a',0.03,'alpha',pi/2,'qlim', deg2rad([-143.5 43.5]), 'offset',0);
            link(3) = Link('d',0.2515,'a',0.0405,'alpha',-pi/2,'qlim', deg2rad([-123.5 80]), 'offset', 0);
            link(4) = Link('d',0,'a',0.0405,'alpha',-pi/2,'qlim',deg2rad([-290 290]),'offset', 0);
            link(5) = Link('d',0.265,'a',0.027,'alpha',-pi/2,'qlim',deg2rad([-88 138]), 'offset',0);
            link(6) = Link('d',0,'a',-0.027,'alpha',pi/2,'qlim',deg2rad([-229 229]), 'offset', 0);
            link(7) = Link('d',0.036,'a',0,'alpha',0,'qlim',deg2rad([-168.5 168.5]), 'offset', 0);

            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
