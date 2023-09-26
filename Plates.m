classdef Plates < handle
    properties(Access = public)
        plates;
        plateStack;
        numOfPlates;
        plateHandles;
        initialTargetTransforms;
        finalTargetTransforms;
    end
    
    methods
        function self = Plates()
            self.platePositions();
            % self.guiObj = GUI(self.bricks, self.brickWall);
            % f = self.guiObj.UIFigure;
            % while isvalid(f)
            %     self.updateBrickPositions();
            %     pause(0.5);
            % end
            self.placePlates();
        end

        function platePositions(self)
            self.plates = {
                [1.90, 2.30, 1.0];
                [1.90, 2.45, 1.0]; 
                [1.90, 2.60, 1.0];
                [1.90, 2.75, 1.0];
                [2.05, 2.30, 1.0]; 
                [2.05, 2.45, 1.0];
                [2.05, 2.60, 1.0]; 
                [2.05, 2.75, 1.0];
            };

            self.plateStack = {
                [1.0, 2.0 1.00];
                [1.0, 2.0 1.01];
                [1.0, 2.3 1.02];
                [1.0, 2.3 1.00];
                [1.0, 2.3 1.01];
                [1.0, 2.6 1.02];
                [1.0, 2.6 1.00];
                [1.0, 2.6 1.01];
            };
        end

        function placePlates(self)
            self.numOfPlates = length(self.plates);
            self.plateHandles = zeros(1, self.numOfPlates);

            % Convert initial plate locations to target transforms
            self.initialTargetTransforms = cell(size(self.plates));
            for i = 1:self.numOfPlates
                translation = self.plates{i}(1:3);

                % Create the homogeneous transformation matrix
                t = transl(translation);

                self.initialTargetTransforms{i} = t;
            end

            % Convert final plate locations to target transforms
            self.finalTargetTransforms = cell(size(self.plates));
            for i = 1:self.numOfPlates             
                translation = self.plateStack{i}(1:3);

                % Create the homogeneous transformation matrix
                t = transl(translation);

                self.finalTargetTransforms{i} = t;
            end

            % Loop through each plate location and place a plate there
            for i = 1:length(self.plates)
                % Retrieve the 1x6 array from the cell
                xyz = self.plates{i}(1:3);

                % Place the plate at the specified location
                h = PlaceObject('plateRed.ply', xyz);
                self.plateHandles(i) = h;

                % Translate to origin, rotate, then translate back
                verts = get(h, 'Vertices');
                verts = verts - xyz;  % Translate to origin
                verts = [verts, ones(size(verts, 1), 1)] * trotx(pi/2);  % Rotate the plates to stand vertically
                verts = verts(:, 1:3) + xyz;  % Translate back to original position
                set(h, 'Vertices', verts);
            end
        end
    end
end