classdef Environment < handle
    properties(Access = public)
        light1
        light2
        flashingLight1
        flashingLight2
    end
    
    methods
        function self = Environment()
            self.initialiseEnvironment();
        end
        
        function initialiseEnvironment(self)
            % Set up the environment
            axis([-5 5 -5 5 0 2.2]);
            
            % Load and display the kitchen model
            self.displayModel('KitchenNew.ply');
            
            % Add and rotate light at position 1 (180 degrees around Z-axis)
            self.light1 = self.addLight('Light.ply', [-1.7, 3, 1.6], 180);
            
            % Add and rotate light at position 2 (90 degrees around Z-axis)
            self.light2 = self.addLight('Light.ply', [2.1, 3.25, 1.6], 90);
            
            % Place the kitchen flooring
            self.placeFlooring();
        end

        function displayModel(self, plyFile)
            [f, v, data] = plyread(plyFile, 'tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            trisurf(f, v(:,1), v(:,2), v(:,3), ...
                'FaceVertexCData', vertexColours, ...
                'EdgeColor', 'interp', ...
                'EdgeLighting', 'flat');
            hold on;
        end
        
        function lightMesh_h = addLight(self, plyFile, position, rotationAngle)
            [f, v, data] = plyread(plyFile, 'tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            % Translate and rotate the vertices
            v = self.rotateVertices(v, rotationAngle);
            v = self.translateVertices(v, position);

            % Plot the light
            lightMesh_h = trisurf(f, v(:,1), v(:,2), v(:,3), ...
                                  'FaceVertexCData', vertexColours, ...
                                  'EdgeColor', 'interp', ...
                                  'EdgeLighting', 'flat');
            hold on;
        end
        
        function vertices = rotateVertices(~, vertices, angleDegrees)
            angleRadians = deg2rad(angleDegrees);
            rotationMatrix = [cos(angleRadians), -sin(angleRadians), 0;
                              sin(angleRadians),  cos(angleRadians), 0;
                              0,                 0,                1];
            vertices = (rotationMatrix * vertices')';
        end
        
        function vertices = translateVertices(~, vertices, translation)
            vertices = vertices + translation;
        end
        
        function placeFlooring(self)
            surf([-5,-5;5,5], ...
                 [-3,3.5;-3,3.5], ...
                 [0.01,0.01;0.01,0.01], ...
                 'CData', imread('KitchenFlooring.jpg'), ...
                 'FaceColor', 'texturemap');
            hold on;
        end

        % Impletemting the flashing light in the simulation
        % function toggleFlashingLight(self, robotNum, condition)
        %     if robotNum == 1
        %         if condition
        %             if isempty(self.flashingLight1) || ~isvalid(self.flashingLight1)
        %                 self.flashingLight1 = self.addLight('LightOnYellow.ply', [-1.7, 3, 1.6], 180);
        %             end
        %         else
        %             if ~isempty(self.flashingLight1) && isvalid(self.flashingLight1)
        %                 delete(self.flashingLight1);
        %                 self.flashingLight1 = [];
        %             end
        %         end
        % 
        %     elseif robotNum == 2
        %         if condition
        %             if isempty(self.flashingLight2) || ~isvalid(self.flashingLight2)
        %                 self.flashingLight2 = self.addLight('LightOnYellow.ply', [2.1, 3.25, 1.6], 90);
        %             end
        %         else
        %             if ~isempty(self.flashingLight2) && isvalid(self.flashingLight2)
        %                 delete(self.flashingLight2);
        %                 self.flashingLight2 = [];
        %             end
        %         end
        %     end
        % end
    end
end