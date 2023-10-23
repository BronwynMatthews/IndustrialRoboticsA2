classdef InitialisePlateStackers < handle
    properties(Access = public)
        plateStackerStart;
        plateStackerFinal;
        numofPlateStackers = 3;

    end
    
    % Constructor
    methods
        function self = InitialisePlateStackers()
            self.plateStackerPositions();
            self.placePlatesStacker();

        end

        % below is code to give the plate stackers a position for their
        % start and end position
        function plateStackerPositions(self)
            self.plateStackerStart = {
                [1.00, 3.00, 1.00];
                [1.15, 2.85, 1.00];
                [1.30, 2.70, 1.00];
            }; 

            self.plateStackerFinal = {
                [0.5, 2.2, 1.4];
                [0.0, 2.2, 1.4];
                [-0.5, 2.2, 1.4];
            };
        end

        function placePlatesStacker(self)
            self.numOfPlatesStacker = length(self.plateStackerLocations);

%%%%%%%%%%%%%%%%%%%%%%% PLEASE HELP ALEX %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Convert initial and final plate locations to target transforms
            self.initialTargetTransforms = cell(size(self.plateStackerLocations));
            self.safeInitialTargetTransforms = cell(size(self.plateStackerLocations));
            self.stackTargetTransforms = cell(size(self.plateStackerLocations));
            self.safeStackTargetTransforms = cell(size(self.plateStackerLocations));
            self.finalTargetTransforms = cell(size(self.plateStackerLocations));
            self.safeFinalTargetTransforms = cell(size(self.plateStackerLocations));

            for i = 1:self.numOfPlatesStacker
                translationInitial = self.plateStackerLocations{i}(1:3);
                translationFinal = self.plateStackerFinal{i}(1:3);

                % Create the homogeneous transformation matrix
                tInitial = transl(translationInitial);
                tStack = transl(translationStack);
                tFinal = transl(translationFinal);

                self.initialTargetTransforms{i} = tInitial;
                self.safeInitialTargetTransforms{i} = tInitial;
                self.safeInitialTargetTransforms{i}(3,4) = self.safeInitialTargetTransforms{i}(3,4) + self.safeOffset;

                self.stackTargetTransforms{i} = tStack;
                self.safeStackTargetTransforms{i} = tStack;
                self.safeStackTargetTransforms{i}(3,4) = self.safeStackTargetTransforms{i}(3,4) + self.safeOffset;

                self.finalTargetTransforms{i} = tFinal;
                self.safeFinalTargetTransforms{i} = tFinal;
                self.safeFinalTargetTransforms{i}(3,4) = self.safeFinalTargetTransforms{i}(3,4) + self.safeOffset;
%%%%%%%%%%%%%%%%%%%%%%% PLEASE HELP ALEX %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            end
        end
    end
end






