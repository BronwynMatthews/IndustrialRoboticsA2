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
                [1.3, 2.5 1.00];
                [1.3, 2.5 1.01];
                [1.3, 2.5 1.02];
                [1.2, 2.7 1.00];
                [1.2, 2.7 1.01];
                [1.3, 2.9 1.02];
                [1.3, 2.9 1.00];
                [1.3, 2.9 1.01];
                [1.3, 2.9 1.02];
            };
        end

        function placePlates(self)
            self.numOfPlates = length(self.plateLocations);
            % self.plateHandles = zeros(1, self.numOfPlates);

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
                self.safeFinalTargetTransforms{i}(3,4) = self.safeFinalTargetTransforms{i}(3,4) + self.safeOffset;
            end

            % Loop through each plate location and place a plate there
            % for i = 1:length(self.plateLocations)
            %     % Retrieve the 1x6 array from the cell
            %     xyz = self.plateLocations{i}(1:3)
            % 
            %     % Place the plate at the specified location
            %     % h = PlaceObject('plateRed.ply', xyz);
            %     % self.plateHandles(i) = h;
            % 
            %     % Translate to origin, rotate, then translate back
            %     % verts = get(h, 'Vertices');
            %     % verts = verts - xyz;  % Translate to origin
            %     % verts = [verts, ones(size(verts, 1), 1)] * trotx(pi/2);  % Rotate the plates to stand vertically
            %     % verts = verts(:, 1:3) + xyz;  % Translate back to original position
            %     % set(h, 'Vertices', verts);
            % 
            %     self.plates{i} = Plate(xyz)
            % 
            % end
            plateNum = 1;

            for i = 1:3
                for j = 1:3
                    if i == 1
                        self.redPos{j} = self.plateLocations{plateNum}(1:3);
                    elseif i == 2
                        self.bluePos{j} = self.plateLocations{plateNum}(1:3);
                    elseif i == 3
                        self.greenPos{j} = self.plateLocations{plateNum}(1:3);
                    end
                    plateNum = plateNum + 1;
                end
            end
        end

        function plateStacker(self)
            initialPlace = [1.3, 2.5, 0.95];
            
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