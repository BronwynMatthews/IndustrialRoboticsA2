classdef Rectangles < handle

    properties
        rectangles
    end

    methods 
        function self = Rectangles()
        % Define the rectangles here
        self.rectangles = {
            struct('lower', [-2.5, 3.25, 0], 'upper', [4, 3.6, 2.1], 'plotOptions', struct('plotVerts', true, 'plotEdges', true, 'plotFaces', true)), % wall
            struct('lower', [-1.1, 1.5, 1.40], 'upper', [1.2, 2.15, 1.45], 'plotOptions', struct('plotVerts', true, 'plotEdges', true, 'plotFaces', true)) % Plate stacker bench location
            struct('lower', [-1.7, 1, 0.9], 'upper', [1.7, 3.25, 1], 'plotOptions', struct('plotVerts', true, 'plotEdges', true, 'plotFaces', true)) % Bench both robots mounted too
            struct('lower', [-3.25, 2.3, 0], 'upper', [-1.75, 3.15, 1.95], 'plotOptions', struct('plotVerts', true, 'plotEdges', true, 'plotFaces', true)) % Plate stacker bench location
            
        };
        end
        
        function draw(self, axis_h)
            if nargin < 2
                axis_h = gca;
            end
            hold(axis_h, 'on');
            
            for i = 1:length(self.rectangles)
                rectangle = self.rectangles{i};
                self.RectangularPrism(rectangle.lower, rectangle.upper, rectangle.plotOptions, axis_h);
            end
        end    
    
        
        function [vertex, face, faceNormals] = RectangularPrism(self, lower, upper, plotOptions, axis_h)

            if nargin < 4
                axis_h = gca;
                if nargin < 3
                    plotOptions.plotVerts = false;
                    plotOptions.plotEdges = true;
                    plotOptions.plotFaces = true;
                end
            end
            hold(axis_h, 'on');


            vertex = [lower(1), lower(2), lower(3);
                      upper(1), lower(2), lower(3);
                      upper(1), upper(2), lower(3);
                      lower(1), upper(2), lower(3);
                      lower(1), lower(2), upper(3);
                      upper(1), lower(2), upper(3);
                      upper(1), upper(2), upper(3);
                      lower(1), upper(2), upper(3)];
        
            face = [1, 2, 6, 5;
                    2, 3, 7, 6;
                    3, 4, 8, 7;
                    4, 1, 5, 8;
                    1, 2, 3, 4;
                    5, 6, 7, 8];
        
            if 2 < nargout
                faceNormals = zeros(size(face, 1), 3);
                for faceIndex = 1:size(face, 1)
                    v1 = vertex(face(faceIndex, 1)', :);
                    v2 = vertex(face(faceIndex, 2)', :);
                    v3 = vertex(face(faceIndex, 3)', :);
                    faceNormals(faceIndex, :) = unit(cross(v2 - v1, v3 - v1));
                end
            end
        
            %% Plotting options
            if isfield(plotOptions, 'plotVerts') && plotOptions.plotVerts
                for i = 1:size(vertex, 1)
                    plot3(axis_h, vertex(i, 1), vertex(i, 2), vertex(i, 3), 'r*');
                    text(axis_h, vertex(i, 1), vertex(i, 2), vertex(i, 3), num2str(i));
                end
            end
        
            if isfield(plotOptions, 'plotEdges') && plotOptions.plotEdges
                links = [1, 2;
                         2, 3;
                         3, 4;
                         4, 1;
                         1, 5;
                         2, 6;
                         3, 7;
                         4, 8;
                         5, 6;
                         6, 7;
                         7, 8;
                         8, 5];
        
                for i = 1:size(links, 1)
                    plot3(axis_h, [vertex(links(i, 1), 1), vertex(links(i, 2), 1)], ...
                                  [vertex(links(i, 1), 2), vertex(links(i, 2), 2)], ...
                                  [vertex(links(i, 1), 3), vertex(links(i, 2), 3)], 'k');
                end
            end
        
            if isfield(plotOptions, 'plotFaces') && plotOptions.plotFaces
                tcolor = [0, 0, 1];
                patch('Faces', face, 'Vertices', vertex, 'FaceVertexCData', tcolor, 'FaceColor', 'flat', 'lineStyle', 'none', 'Parent', axis_h);
            end
        end
    end
end
