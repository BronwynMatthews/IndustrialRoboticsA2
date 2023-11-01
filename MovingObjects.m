classdef MovingObjects < handle
    properties
        Position    
        Radius      % Radius of the object (person)
        IsInDanger  % Boolean dictating if the person was too close
    end

    methods
        function self = MovingObjects(position, radius)
            
            self.Position = position;
            self.Radius = radius;
            self.IsInDanger = false;
        end

        function updatePosition(self, newPosition)
            
            T1 = transl(0,0,10);
            
            self.Position = newPosition;

        end

        function checkSafety(self, lightCurtain)

            % something like below where we can integrate a distance calc
            distances = sqrt(sum((lightCurtain - self.Position).^2, 2));

            % Check if any of the distances are less than the safety radius
            % (safety raidus is the person's radius) 
            if any(distances < self.Radius)
                self.IsInDanger = true; % implement something from this line within the main
                disp('Warning: Person is within the light curtain! Cease operations.');
            else
                self.IsInDanger = false;
            end
        end
    end
end