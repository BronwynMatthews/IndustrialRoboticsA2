classdef InitialisePlates < handle
    properties(Access = public)
        plates;
        plateLocations;
        redPos;
        bluePos;
        greenPos;
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
        function self = InitialisePlates()
            self.platePositions();
            % self.guiObj = GUI(self.bricks, self.brickWall);
            % f = self.guiObj.UIFigure;
            % while isvalid(f)
            %     self.updateBrickPositions();
            %     pause(0.5);
            % end
            self.placePlates();
            self.plateStacker();

        end

        function platePositions(self)
            self.plateLocations = {
                [1.90, 2.40, 1.00];
                [1.90, 2.45, 1.00]; 
                [1.90, 2.50, 1.00];
                [1.90, 2.55, 1.00];
                [1.90, 2.60, 1.00];
                [2.05, 2.55, 1.00]; 
                [2.05, 2.60, 1.00];
                [2.05, 2.65, 1.00]; 
                [2.05, 2.70, 1.00];
            };

            self.plateStack = {
                [1.3, 2.5 1.05];
                [1.3, 2.5 1.06];
                [1.3, 2.5 1.07];
                [1.1, 2.7 1.05];
                [1.1, 2.7 1.06];
                [1.3, 2.9 1.07];
                [1.3, 2.9 1.05];
                [1.3, 2.9 1.06];
                [1.3, 2.9 1.07];
            };
        end

        function placePlates(self)
            self.numOfPlates = length(self.plateLocations);

            % Convert initial and final plate locations to target transforms
            self.initialTargetTransforms = cell(size(self.plateLocations));
            self.safeInitialTargetTransforms = cell(size(self.plateLocations));
            self.finalTargetTransforms = cell(size(self.plateLocations));
            self.safeFinalTargetTransforms = cell(size(self.plateLocations));

            for i = 1:self.numOfPlates
                translationInitial = self.plateLocations{i}(1:3);
                translationFinal = self.plateStack{i}(1:3);

                % Create the homogeneous transformation matrix
                tInitial = transl(translationInitial);
                tFinal = transl(translationFinal);

                self.initialTargetTransforms{i} = tInitial;
                self.safeInitialTargetTransforms{i} = tInitial;
                self.safeInitialTargetTransforms{i}(3,4) = self.safeInitialTargetTransforms{i}(3,4) + self.safeOffset;

                self.finalTargetTransforms{i} = tFinal;
                self.safeFinalTargetTransforms{i} = tFinal;
                self.finalTargetTransforms{i}(3,4) = self.finalTargetTransforms{i}(3,4) - 0.24;
                self.safeFinalTargetTransforms{i}(3,4) = self.safeFinalTargetTransforms{i}(3,4) + self.safeOffset;
            end
        end

        function plateStacker(self)
            initialPlace = [1.225, 2.375, 0.95];
            
            p = PlaceObject('plateStacker.ply', initialPlace);

            % Unsure if below is required as it sets verts?? Left it in
            % incase
            % verts = get(p, 'Vertices');
            % verts = verts - initialPlace;  % Translate to origin
            % verts = verts(:, 1:3) + initialPlace;  % Translate back to original position
            % set(p, 'Vertices', verts);
        end

    end
end