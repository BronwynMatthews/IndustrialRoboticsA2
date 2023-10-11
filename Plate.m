classdef Plate < RobotBaseClass
    properties(Access = public)              
        plyFileNameStem = 'plateRed'
    end
    
    methods
%% Define robot Function 
        function self = Plate(baseTr, colour)
       
            self.CreateModel();

            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotz(pi/2) * troty(pi/2);

            if strcmp(colour, 'red')
                self.plyFileNameStem = 'plateRed';
            elseif strcmp(colour,'blue')
                self.plyFileNameStem = 'plateBlue';
            elseif strcmp(colour,'green')
                self.plyFileNameStem = 'plateGreen';
            end
           
            self.PlotAndColourRobot();    
            
        end


%% Create the plate model
        function CreateModel(self)   
            link(1) = Link('alpha',0,'a',0,'d',0,'offset',0);
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end