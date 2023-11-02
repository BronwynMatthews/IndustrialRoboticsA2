classdef LightCurtains < handle

    properties
        normals;
        midPoints;
        xLims = [1.8, 3; 1, 1.75; -1.75, -1]; % limits of lightcurtain 1
        yLims = [1, 1.9; 1.5, 1.5; 1.5, 2.3]    % limits of light curtain 2
        zLims = [0, 2; 1, 2; 1, 2]  %limits of light curtain 3
    end

    methods 
        function self = LightCurtains()
            self.normals = zeros(3, 3);

            self.midPoints = zeros(3, 3);

            for i = 1:3
                [tempX, tempY, tempZ] = self.normalVector(self.xLims(i,1), self.xLims(i,2), self.yLims(i,1), self.yLims(i,2));

                self.normals(i,1) = tempX;
                self.normals(i,2) = tempY;
                self.normals(i,3) = tempZ;

                [X, Y, Z] = self.MidPoint(self.xLims(i,1), self.xLims(i,2), self.yLims(i,1), self.yLims(i,2), self.zLims(i,1), self.zLims(i,2));

                self.midPoints(i,1) = X;
                self.midPoints(i,2) = Y;
                self.midPoints(i,3) = Z;
            end

        end    

        % Calculates the normal of the light curtains
        function [Nx, Ny, Nz] = normalVector(self, px1, px2, py1, py2)
            Nx = px1 - px2;
            Ny = py1 - py2;
            Nz = 0;
        end

        function [x, y, z] = MidPoint(self, px1, px2, py1, py2, pz1, pz2)
            % make array of midpoints
            % Calculate midpoints for each set of limits
            x = (px1 + px2)/2;
            y = (py1 + py2)/2;
            z = (pz1 + pz2)/2;
        end
    end
end