classdef Plate < handle
    
    %#ok<*TRYNC>    

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
    end
    
    properties
        %> Number of cows
        plateCount = 1;
        
        %> A cell structure of \c cowCount cow models
        plateModel;
        platePos;
        workspaceDimensions;
    end
    
    methods
        %% ...structors
        function self = Plate(position)
            if 0 < nargin
                self.platePos = position;
            end

            self.workspaceDimensions = [-5, 5, -5, 5, 0, 2];

            % Create the required number of plates
            self.plateModel = self.GetPlateModel(['plateRed',1]);
            % Random spawn
            basePose = transl(position)

            self.plateModel.base = basePose;
            
             % Plot 3D model
            plot3d(self.plateModel,0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist');
            % Hold on after the first plot (if already on there's no difference)

            axis equal
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end 
        end
        
        function delete(self)
            handles = findobj('Tag', self.plateModel.name);
            h = get(handles,'UserData');
            try delete(h.robot); end
            try delete(h.wrist); end
            try delete(h.link); end
            try delete(h); end
            try delete(handles); end
        end    
    end
    
    methods (Static)
        %% GetPLateModel
        function model = GetPlateModel(name)
            if nargin < 1
                name = 'plateRed';
            end
            [faceData,vertexData] = plyread('plateRed.ply','tri');
            link1 = Link('alpha',pi/2,'a',0,'d',0.1,'offset',0);
            model = SerialLink(link1,'name',name);
            
            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end
    end    
end