classdef Plates < handle
    properties(Access = public)
        plates;
        plateStack;
        numOfPlates;
        plateHandles;
        initialTargetTransforms;
        safeInitialTargetTransforms;
        finalTargetTransforms;
        safeFinalTargetTransforms;
        safeOffset = 0.3;
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
                [1.90, 2.40, 1.00];
                [1.90, 2.45, 1.00]; 
                [1.90, 2.50, 1.00];
                [1.90, 2.55, 1.00];
                [2.05, 2.55, 1.00]; 
                [2.05, 2.60, 1.00];
                [2.05, 2.65, 1.00]; 
                [2.05, 2.70, 1.00];
            };

            self.plateStack = {
                [1.3, 2.5 1.00];
                [1.3, 2.5 1.01];
                [1.3, 2.5 1.02];
                [1.2, 2.7 1.00];
                [1.2, 2.7 1.01];
                [1.3, 2.9 1.02];
                [1.3, 2.9 1.00];
                [1.3, 2.9 1.01];
            };
        end

        function placePlates(self)
            self.numOfPlates = length(self.plates);
            self.plateHandles = zeros(1, self.numOfPlates);

            % Convert initial and final plate locations to target transforms
            self.initialTargetTransforms = cell(size(self.plates));
            self.safeInitialTargetTransforms = cell(size(self.plates));
            self.finalTargetTransforms = cell(size(self.plates));
            self.safeFinalTargetTransforms = cell(size(self.plates));

            for i = 1:self.numOfPlates
                translationInitial = self.plates{i}(1:3);
                translationFinal = self.plateStack{i}(1:3);

                % Create the homogeneous transformation matrix
                tInitial = transl(translationInitial);
                tFinal = transl(translationFinal);

                self.initialTargetTransforms{i} = tInitial;
                self.safeInitialTargetTransforms{i} = tInitial;
                self.safeInitialTargetTransforms{i}(3,4) = self.safeInitialTargetTransforms{i}(3,4) + self.safeOffset;

                self.finalTargetTransforms{i} = tFinal;
                self.safeFinalTargetTransforms{i} = tFinal;
                self.safeFinalTargetTransforms{i}(3,4) = self.safeFinalTargetTransforms{i}(3,4) + self.safeOffset;
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