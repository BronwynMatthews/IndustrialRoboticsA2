classdef Environment < handle
    properties(Access = public)

    end
    
    methods
        function self = Environment()
            self.initialiseEnvironment();
        end
        
        function initialiseEnvironment(self)
            % grid on
            % axis on
            axis([-5 5 -5 5 0 2.2]);
            % hold on

            [f,v,data] = plyread('Kitchen.ply','tri');
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            % Then plot the trisurf
            tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            % PlaceObject('Kitchen4.ply', [0,0,0])

            surf([-5,-5;5,5] ...
            ,[-3,3.5;-3,3.5] ...
            ,[0.01,0.01;0.01,0.01] ...
            ,'CData',imread('KitchenFlooring.jpg') ...
            ,'FaceColor','texturemap');
        end
    end
end