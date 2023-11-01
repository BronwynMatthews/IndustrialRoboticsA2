function result = IsCollision(robot, qMatrix, collisionPoints, ~)
    if nargin < 4
        collisionThreshold = 0.02; % default collision threshold
    end
    
    result = false;
    
    for qIndex = 1:size(qMatrix,1)
        tr = GetRobotTransforms(robot, qMatrix(qIndex, :));

        for linkIndex = 1:size(tr, 3)-1
            linkStart = tr(1:3, 4, linkIndex);
            linkEnd = tr(1:3, 4, linkIndex + 1);
            
            for rectIndex = 1:length(collisionPoints.rectangles)
                rectangle = collisionPoints.rectangles{rectIndex};
                [vertex, face, faceNormals] = collisionPoints.RectangularPrism(rectangle.lower, rectangle.upper, rectangle.plotOptions);
                
                for faceIndex = 1:size(face,1)
                    vertOnPlane = vertex(face(faceIndex,1)',:);
                    [intersectP, check] = LinePlaneIntersection(faceNormals(faceIndex,:), vertOnPlane, linkStart', linkEnd');
                    
                    if check == 1 && IsIntersectionPointInsideTriangle(intersectP, vertex(face(faceIndex,:)',:))
                        plot3(intersectP(1), intersectP(2), intersectP(3), 'r*');
                        disp('Collision Detected');
                        result = true;
                        return;
                    end
                end
            end
        end
    end
end

function tr = GetRobotTransforms(robot, q)
    tr = zeros(4, 4, robot.n+1);
    tr(:,:,1) = robot.base;
    L = robot.links;
    for i = 1:robot.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(q(i) + L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end
end