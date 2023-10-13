classdef Panda < RobotBaseClass
    properties(Access = public)   
        plyFileNameStem = 'Panda';
        q0 = [0,-pi/3,0,pi,0,0,0]
    end
    
    methods
%% Constructor
function self = Panda(baseTr)
			self.CreateModel();
            if nargin < 1			
                baseTr = eye(4);
            end

            self.model.base = self.model.base.T * baseTr;
            
            self.PlotAndColourRobot();
            
            % self.model.teach();
            % self.model.teach(q0)
           
            % self.model.plot(self.q0);  
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.333,'a',0,'alpha',-pi/2,'qlim',[-2.7437 2.7437]);  
            link(2) = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-1.7837 1.7837]);  
            link(3) = Link('d',0.316,'a',0.0825,'alpha',pi/2,'qlim',[-2.9007 2.9007]);  
            link(4) = Link('d',0,'a',-0.0825,'alpha',-pi/2,'qlim',[-3.0421 -0.1518]);  
            link(5) = Link('d',0.384,'a',0,'alpha',-pi/2,'qlim',[-2.8065 2.8065]);  
            link(6) = Link('d',0,'a',0.088,'alpha',pi/2,'qlim',[0.5445 4.5169]);  
            link(7) = Link('d',-0.1347,'a',0,'alpha',pi,'qlim',[-3.0159 3.0159]);  
            % link(7) = Link('d',-0.0107,'a',0,'alpha',pi/2,'qlim',[-3.0159 3.0159]);  

            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
