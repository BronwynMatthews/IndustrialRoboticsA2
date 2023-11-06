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
        stackerTransforms;
        safeOffset = 0.3;
        plateOffset = 0.12;
        stackers;
        ur5GripperOffset = 0.25; % OFFSET FOR END EFFECTOR/PLATES
    end
    
    methods
        function self = InitialisePlates()
            self.PlatePositions();
            self.PlateStacker();
            self.PlateTransforms();
        end

        function PlatePositions(self)

            platesXYZ = {
                [0.8, 2.9, 1.04];
                [0.95, 2.55, 1.04];
                [1.3, 2.4, 1.04];
                
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

            self.plateFinal = {
                [0.5, 1.95, 1.45];
                [0.0, 1.95, 1.45];
                [-0.5, 1.95, 1.45];
            };
        end

        function PlateTransforms(self)
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

        function PlateStacker(self)
            stack = 1;
            for i = 1:3
                pos = self.plateStack{stack}(1:3);
                pos(3) = pos(3) - 0.09;
                self.stackers{i} = PlaceObject('plateStacker.ply', pos);
                stack = stack + 3;
            end
        end

        function transforms = PlateStackerTransforms(self, ur5, stackCounter, stackModel)
            transforms = cell(7);
            transforms{1} = stackModel.model.base.T;
            if stackCounter == 1
                transforms{1}(1,4) = transforms{1}(1,4) - self.ur5GripperOffset - 0.15;
                transforms{1} = transforms{1} * rpy2tr(0, -90, 90, 'deg');  

                transforms{2} = transforms{1};
                transforms{2}(1,4) = transforms{2}(1,4) + 0.15;

                transforms{3} = transforms{2};
                transforms{3} = transforms{2} * rpy2tr(-90, 0, 0, 'deg');
            else
                transforms{1}(2,4) = transforms{1}(2,4) + self.ur5GripperOffset + 0.15;
                transforms{1} = transforms{1} * rpy2tr(0, -90, 0, 'deg');  

                transforms{2} = transforms{1};
                transforms{2}(2,4) = transforms{2}(2,4) - 0.15;

                transforms{3} = transforms{2};
            end
            transforms{3}(3,4) = transforms{3}(3,4) + 0.45;

            transforms{4} = self.finalTargetTransforms{stackCounter} * rpy2tr(0, -90, 90, 'deg');
            transforms{4}(2,4) = transforms{4}(2,4) + self.ur5GripperOffset + 0.25;

            transforms{5} = transforms{4};
            transforms{5}(2,4) = transforms{5}(2,4) - 0.25;

            transforms{6} = transforms{4};

            transforms{7} = ur5.model.fkine(ur5.q0).T;

        end
    end
end