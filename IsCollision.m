% does this need to be a class, thinking of leaving it as a function within
% the space to be called upon?? 

function result = IsCollision(robot, qMatrix, objectPoints, collisionThreshold)
    if nargin < 4
        collisionThreshold = 0.02; % collision threshold
    end
    result = false;
    
    % test if the robot (cycling through the links) is 
    for qIndex = 1:size(qMatrix,1)
        tr = GetLinkPoses(qMatrix(qIndex,:), robot);

        for i = 1:size(tr, 3)
            linkPoint = tr(1:3, 4, i);
            
            % loop through array of object points (might change this)
            for j = 1:size(objectPoints, 1)
                objectPoint = objectPoints(j, :);
                distance = norm(linkPoint - objectPoint');
                
                if distance <= collisionThreshold
                    result = true;
                    return;
                end
            end
        end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% how we will implement this 
% robot = % (robot model that is being checked for collisions)
% qMatrix = % (trajectory)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    end
end