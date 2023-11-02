function [vertex, face, faceNormals] = RectangularPrism(lower, upper)

    plotOptions.plotVerts = false;
    plotOptions.plotEdges = true;
    plotOptions.plotFaces = true;
    axis_h = gca;
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