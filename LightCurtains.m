classdef LightCurtains < handle

    properties
        lightcurtains
    end

    methods 
        function self = LightCurtains()
            % Define the planes of the light curtains
            self.lightcurtains = {
                struct('lower', [3, 1.9, 0], 'upper', [1, 1.2, 2], 'plotOptions', struct('plotVerts', true, 'plotEdges', true, 'plotFaces', true)),
                struct('lower', [-1.5, 2.3, 0], 'upper', [3, 2.4, 1.5], 'plotOptions', struct('plotVerts', true, 'plotEdges', true, 'plotFaces', true))
                % three different light curtains
            };
        end

        function draw(self, axis_h)
            if nargin < 2
                axis_h = gca;
            end
            hold(axis_h, 'on');
            
            for i = 1:length(self.lightcurtains)
                plane = self.lightcurtains{i};
                self.LightPlane(plane.lower, plane.upper, plane.plotOptions, axis_h);
            end
        end    

        function LightPlane(self, lower, upper, plotOptions, axis_h)
            if nargin < 4
                axis_h = gca;
                if nargin < 3
                    plotOptions.plotVerts = false;
                    plotOptions.plotEdges = true;
                    plotOptions.plotFaces = true;
                end
            end
            hold(axis_h, 'on');

            % Define the corners of the plane
            vertex = [lower(1), lower(2), lower(3);
                      upper(1), lower(2), lower(3);
                      upper(1), upper(2), lower(3);
                      lower(1), upper(2), lower(3)];

            % Define the faces of the plane
            face = [1, 2, 3, 4];
        
            % Plot vertices
            if isfield(plotOptions, 'plotVerts') && plotOptions.plotVerts
                for i = 1:size(vertex, 1)
                    plot3(axis_h, vertex(i, 1), vertex(i, 2), vertex(i, 3), 'r*');
                    text(axis_h, vertex(i, 1), vertex(i, 2), vertex(i, 3), num2str(i));
                end
            end

            % Plot edges
            if isfield(plotOptions, 'plotEdges') && plotOptions.plotEdges
                links = [1, 2;
                         2, 3;
                         3, 4;
                         4, 1];
        
                for i = 1:size(links, 1)
                    plot3(axis_h, [vertex(links(i, 1), 1), vertex(links(i, 2), 1)], ...
                                  [vertex(links(i, 1), 2), vertex(links(i, 2), 2)], ...
                                  [vertex(links(i, 1), 3), vertex(links(i, 2), 3)], 'k');
                end
            end

            % Plot faces
            if isfield(plotOptions, 'plotFaces') && plotOptions.plotFaces
                tcolor = [1, 1, 0];  % RGB values for yellow
                patch('Faces', face, 'Vertices', vertex, 'FaceVertexCData', tcolor, ...
              'FaceColor', 'flat', 'EdgeColor', 'k', 'LineWidth', 1.5, ...
              'FaceAlpha', 1, 'Parent', axis_h);
            end
        end
    end
end