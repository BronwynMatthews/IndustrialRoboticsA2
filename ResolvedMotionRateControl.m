function qMatrix = ResolvedMotionRateControl(model, tr1, plane)
        robot = model;
        q = robot.model.getpos();

        n = robot.model.n;
        lims = robot.model.qlim;
        velocity = 0.25;
        deltaT = 0.050;  

        rpy = tr2rpy(tr1);
        
        q0 = robot.model.fkine(q);
        q0 = q0.T;
        startPoint = q0(1:3,4)';
        endPoint = tr1(1:3,4)';
        dist = norm(endPoint - startPoint);
        t = dist/velocity;
        % steps = floor(t/deltaT)   % No. of steps for simulation
        steps = 10;
        delta = 2*pi/steps; % Small angle change
        epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
        W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
        
        % 1.2) Allocate array data
        m = zeros(steps,1);             % Array for Measure of Manipulability
        qMatrix = zeros(steps,n);       % Array for joint anglesR
        qdot = zeros(steps,n);          % Array for joint velocities
        theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
        x = zeros(3,steps);             % Array for x-y-z trajectory
        positionError = zeros(3,steps); % For plotting trajectory error
        angleError = zeros(3,steps);    % For plotting trajectory error
        
        % 1.3) Set up trajectory, initial pose
        s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
        for i=1:steps
            if strcmp(plane, 'horizontal')
                x(1,i) = (1-s(i))*startPoint(1) + s(i)*endPoint(1);  % Linear interpolation in x
                x(2,i) = (1-s(i))*startPoint(2) + s(i)*endPoint(2);  % Linear interpolation in y
                x(3,i) = startPoint(3);  % Constant z-height
            elseif strcmp(plane, 'vertical')
                x(1,i) = startPoint(1);  % Constant x
                x(2,i) = startPoint(2); % Constant y
                x(3,i) = (1-s(i))*startPoint(3) + s(i)*endPoint(3);  % Constant z-height
            elseif strcmp(plane, 'x')
                x(1,i) = startPoint(1);  % Constant x
                x(2,i) = (1-s(i))*startPoint(2) + s(i)*endPoint(2); % Linear interpolation in y
                x(3,i) = (1-s(i))*startPoint(3) + s(i)*endPoint(3);  % Linear interpolation in z
            elseif strcmp(plane, 'y')
                x(1,i) = (1-s(i))*startPoint(1) + s(i)*endPoint(1);  % Linear interpolation in x
                x(2,i) = startPoint(2); % Constant y
                x(3,i) = (1-s(i))*startPoint(3) + s(i)*endPoint(3);  % Linear interpolation in z
            elseif strcmp(plane, 'z')
                x(1,i) = startPoint(1);  % Linear interpolation in x
                x(2,i) = startPoint(2); % Linear interpolation in y
                x(3,i) = (1-s(i))*startPoint(3) + s(i)*endPoint(3);  % Constant z-height
            end
            theta(1,i) = rpy(1);          % Constant Roll angle 
            theta(2,i) = rpy(2);     % Constant Pitch angle
            theta(3,i) = rpy(3);          % Constant Yaw angle
        end
        
        T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
        q0 = zeros(1,n);                                                          % Initial guess for joint angles
        qMatrix(1,:) = robot.model.ikcon(T,q);                                            % Solve joint angles to achieve first waypoint
        
        % 1.4) Track the trajectory with RMRC
        for i = 1:steps-1
            % UPDATE: fkine function now returns an SE3 object. To obtain the 
            % Transform Matrix, access the variable in the object 'T' with '.T'.
            T = robot.model.fkine(qMatrix(i,:)).T;                                           % Get forward transformation at current joint state
            deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
            Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
            Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
            Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
            S = Rdot*Ra';                                                           % Skew symmetric!
            linear_velocity = (1/deltaT)*deltaX;
            angular_velocity = [S(3,2);S(3,1);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
            deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
            xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
            J = robot.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
            m(i) = sqrt(det(J*J'));
            if m(i) < epsilon  % If manipulability is less than given threshold
                lambda = (1 - m(i)/epsilon)*5E-2;
            else
                lambda = 0;
            end
            invJ = inv(J'*J + lambda *eye(n))*J';                                   % DLS Inverse
            qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
            for j = 1:n                                                             % Loop through joints 1 to 6
                if qMatrix(i,j) + deltaT*qdot(i,j) < lims(j,1)                     % If next joint angle is lower than joint limit...
                    qdot(i,j) = 0; % Stop the motor
                elseif qMatrix(i,j) + deltaT*qdot(i,j) > lims(j,2)                 % If next joint angle is greater than joint limit ...
                    qdot(i,j) = 0; % Stop the motor
                end
            end
            qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
            positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
            angleError(:,i) = deltaTheta;                                           % For plotting
        end
    end
