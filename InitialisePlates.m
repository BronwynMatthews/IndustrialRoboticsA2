classdef InitialisePlates < handle
    properties(Access = public)
        plates;
        plateLocations;
        plateFinal;
        redPos;
        bluePos;
        greenPos;
        plateStack;
        numOfPlates;
        plateHandles;
        initialTargetTransforms;
        safeInitialTargetTransforms;
        stackTargetTransforms;
        safeStackTargetTransforms;
        finalTargetTransforms;
        safeFinalTargetTransforms;
        safeOffset = 0.3;
        plateOffset = 0.12;
        stackers;
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
            self.plateStacker();
            self.placePlates();
            

        end

        

        function platePositions(self)

            platesXYZ = {
                [0.8, 2.9, 1.04];
                [0.95, 2.55, 1.04];
                [1.25, 2.4, 1.04];
                
            };
    
            self.plateStack = cell(9,1);
            count = 1;
            for i = 1:3
                for j = 0:2
                    self.plateStack{count} = platesXYZ{i};
                    self.plateStack{count}(3) = self.plateStack{count}(3) + (0.02 * j);
                    count = count + 1;
                end
            end


            self.plateLocations = {
                [1.95, 2.48, 1.00];
                [1.95, 2.52, 1.00]; 
                [1.95, 2.56, 1.00];
                [1.95, 2.60, 1.00];
                [1.95, 2.64, 1.00];
                [1.95, 2.68, 1.00]; 
                [1.95, 2.72, 1.00];
                [1.95, 2.76, 1.00]; 
                [1.95, 2.80, 1.00];
            };

            % self.plateStack = {
            %     [0.95, 3.00, 1.04];
            %     [0.95, 3.00, 1.06];
            %     [0.95, 3.00, 1.08];
            %     [0.85, 2.7, 1.04];
            %     [0.85, 2.7, 1.06];
            %     [0.85, 2.7, 1.08];
            %     [1.13, 2.59, 1.04];
            %     [1.13, 2.59, 1.06];
            %     [1.13, 2.59, 1.08];
            % }; % WOKRING 8/9

            plot3(self.plateStack{1}(1), self.plateStack{1}(2), 1.1, 'x')
            plot3(self.plateStack{4}(1), self.plateStack{4}(2), 1.1, 'x')
            plot3(self.plateStack{7}(1), self.plateStack{7}(2), 1.1, 'x')

            self.plateFinal = {
                [0.5, 1.95, 1.45];
                [0.0, 1.95, 1.45];
                [-0.5, 1.95, 1.45];
            };

            plot3(self.plateFinal{1}(1), self.plateFinal{1}(2), 1.5, 'o')
            plot3(self.plateFinal{2}(1), self.plateFinal{2}(2), 1.5, 'o')
            plot3(self.plateFinal{3}(1), self.plateFinal{3}(2), 1.5, 'o')
        end

        function placePlates(self)
            self.numOfPlates = length(self.plateLocations);

            % Convert initial and final plate locations to target transforms
            self.initialTargetTransforms = cell(size(self.plateLocations));
            self.safeInitialTargetTransforms = cell(size(self.plateLocations));
            self.stackTargetTransforms = cell(size(self.plateLocations));
            self.safeStackTargetTransforms = cell(size(self.plateLocations));
            self.finalTargetTransforms = cell(size(self.plateLocations));
            self.safeFinalTargetTransforms = cell(size(self.plateLocations));

            for i = 1:self.numOfPlates
                self.initialTargetTransforms{i} = transl(self.plateLocations{i}(1:3));
                self.initialTargetTransforms{i}(3,4) = self.initialTargetTransforms{i}(3,4) + self.plateOffset;
                self.safeInitialTargetTransforms{i} = self.initialTargetTransforms{i};
                self.safeInitialTargetTransforms{i}(3,4) = self.safeInitialTargetTransforms{i}(3,4) + self.safeOffset;

                self.stackTargetTransforms{i} = transl(self.plateStack{i}(1:3));
                self.stackTargetTransforms{i}(1,4) = self.stackTargetTransforms{i}(1,4) + self.plateOffset;
                self.safeStackTargetTransforms{i} = self.stackTargetTransforms{i};
                self.safeStackTargetTransforms{i}(3,4) = self.safeStackTargetTransforms{i}(3,4) + self.safeOffset;

                
            end
            for i = 1:3
                self.finalTargetTransforms{i} = transl(self.plateFinal{i}(1:3));
            end
        end

        function plateStacker(self)
            stack = 1;
            for i = 1:3
                pos = self.plateStack{stack}(1:3);
                % pos(1) = pos(1) - 0.07;
                % pos(2) = pos(2) + 0.14;
                pos(3) = pos(3) - 0.09;
                self.stackers{i} = PlaceObject('plateStacker.ply', pos);
                stack = stack + 3;
            end

            % Unsure if below is required as it sets verts?? Left it in
            % incase
            % verts = get(p, 'Vertices');
            % verts = verts - initialPlace;  % Translate to origin
            % verts = verts(:, 1:3) + initialPlace;  % Translate back to original position
            % set(p, 'Vertices', verts);
        end

    end
end