classdef Person < RobotBaseClass
    properties(Access = public)              
        plyFileNameStem = 'person'
    end
    
    methods
%% Define robot Function 
        function self = Person(baseTr)

            self.CreateModel();

            if nargin < 1			
				baseTr = eye(4);	
            end
            

            self.model.base = self.model.base.T * baseTr * trotz(pi/2); %* troty(pi/2);

            self.PlotAndColourRobot();   
            
        end


%% Create the person model
        function CreateModel(self)   
            link(1) = Link('alpha',0,'a',0,'d',0,'offset',0);
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end