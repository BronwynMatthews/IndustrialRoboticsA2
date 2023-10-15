classdef plateStacker < RobotBaseClass
    properties(Access = public)              
        plyFileNameStem = 'plateStackerRobot'
    end
    
    methods
%% Define robot Function 
        function self = plateStacker(baseTr, colour)
       
            self.CreateModel();

            if nargin < 1			
				baseTr = eye(4);	
            end

            % May have to edit below rotational transform based on ply
            % orinetation
            self.model.base = self.model.base.T * baseTr * trotz(pi/2) * troty(pi/2);

            self.PlotAndColourRobot();    
            
        end


%% Create the plate model
        function CreateModel(self)   
            link(1) = Link('alpha',0,'a',0,'d',0,'offset',0);
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end