classdef LabAssignment2 < handle
    %LABASSIGNMENT2 Summary of this class goes here
    %   Detailed explanation goes here
    % Check

    properties
        panda;
        linearUR5;
        objPlates;
        plateCounter = 1;
        plateStackerCounter = 1;
        pandaJointAngles;
        pandaEnd;
        pandaGriperOffset = 0.12; % RADIUS OF A PLATE
        ur5JointAngles;
        ur5End;
        ur5GriperOffset = 0.25;
        pandaState;
        ur5State;
        plates;
        plateModel;
        plateStackerModel;
        plateJoints = [0,0];
        plateStackerJoints = [0,0];
        gui;
        stack;
    end

    methods
        function self = LabAssignment2()
            cla;
            % clf;
            clc;

            hold on

            self.InitialiseRobots();
            self.gui = GUI(self);
            self.gui.UpdateGUI(self);
            self.InitialiseEnvironment();
            self.InitialisePlates();
            % self.StartGui();
            self.RunRobot();
        end

        function InitialiseRobots(self)
            self.panda = Panda(transl(1.6, 3.0, 0.95));

            %%  below change to the new linear ur5 (with gripper attachment)
            self.linearUR5 = LinearUR5custom(transl(0.4, 2.6, 0.95));

            self.UpdateRobots();
        end

        function StartGui(self)
            self.gui = GUI(self);
            tic;
            self.gui.UpdateGUI();
            % while(1)
            %     self.gui.UpdateGUI();
            %     disp(['timestep ', num2str(toc)]);
            %     if self.gui.jog
            %         tr = self.gui.StepRobot()
            %         qMatrix = self.panda.model.ikcon(tr)
            %         self.gui.UpdateSliders(qMatrix)
            %         self.panda.model.animate(qMatrix);
            %         self.gui.jog = false;
            %     else
            %         qMatrix = self.gui.GetLinkValues();
            %         self.panda.model.animate(qMatrix);
            %     end
            %     self.UpdateRobots();
            %     pause(0.5);
            % end
        end

        function UpdateGUI(self)
            self.gui = GUI(self);
            qMatrix = self.pandaJointAngles;
            self.gui.UpdateGUI(self);
            disp(['timestep ', num2str(toc)]);
            if self.gui.jog
                tr = self.gui.StepRobot();
                qMatrix = self.panda.model.ikcon(tr);
                self.gui.UpdateSliders(qMatrix)
                self.panda.model.animate(qMatrix);
                self.gui.jog = false;
            else
                self.gui.UpdateSliders(qMatrix);
                qMatrix = self.gui.GetLinkValues();
                % self.panda.model.animate(qMatrix);
            end
            % self.UpdateRobots();
        end

        function InitialiseEnvironment(self)
            Environment(); % Build the workspace for the robot
        end

        function InitialisePlates(self)
            self.objPlates = InitialisePlates(); % plot the plates in the workspace
            % self.objPlates.placeStacker

            plateNum = 1;
            for i = 1:3
                for j = 1:3
                    baseTr = transl(self.objPlates.plateLocations{plateNum}(1:3));
                    switch i
                        case 1
                            colour = 'red';
                        case 2
                            colour = 'blue';
                        case 3
                            colour = 'green';
                    end
                    self.plates{plateNum} = Plate(baseTr, colour);
                    plateNum = plateNum + 1;
                end
            end
        end


        % function InitialisePlateStacker(self)
        %     self.objPlateStackers = InitialisePlateStacker(); % plot the plates in the workspace
        %
        %     plateStackerNum = 1;
        %
        %     for i = 1:3
        %         for j = 1:3
        %             baseTr = self.objPlateStackers.initialTargetTransforms{plateStackerNum};
        %             switch i
        %                 case 1
        %                     colour = 'red';
        %                 case 2
        %                     colour = 'blue';
        %                 case 3
        %                     colour = 'green';
        %             end
        %             self.plates{plateStackerNum} = Plate(baseTr, colour);
        %             plateStackerNum = plateStackerNum + 1;
        %         end
        %     end
        % end


        function RunRobot(self)
            % input('Press enter to begin')
            self.UpdateRobots();
            stackCounter = 1;
            % Initialise log variable
            logData = struct('Time', [], 'Status', [], 'Transform', []); % Create the logData Structure

            % Log the start time and status
            logData.Time = {};
            logData.Time{end+1} = datestr(datetime('now'), 'yyyy-mm-dd HH:MM:SS'); % Log the time that the robot starts
            logData.Status{end+1} = 'Task Started';
            logData.Transform{end+1} = 'N/A';

            % pause(4);

            for i = 1:self.objPlates.numOfPlates
                disp(['Panda unstacking plate ', num2str(i)])
                self.pandaState = 1;
                self.ur5State = 1;
                self.plateModel = self.plates{self.plateCounter};


                % self.movePanda();
                for j = 1:6
                    % STATE 1 is panda moving to safe position above initial plate position (WITHOUT plate)
                    % STATE 2 is panda picking up plate from initial plate position
                    % STATE 3 is panda moving to safe position above initial plate position (WITH plate)
                    % STATE 4 is panda moving to safe position above final plate position (WITH plate)
                    % STATE 5 is panda placing plate in its final position
                    % STATE 6 is panda moving to safe position above final plate position (WITHOUT plate)

                    % Log the start time and status
                    logData.Time = {};
                    logData.Time{end+1} = datestr(datetime('now'), 'yyyy-mm-dd HH:MM:SS'); % Log the time that the robot starts
                    logData.Status{end+1} = ['Panda picking/placing plate ', num2str(i), ' in State ', num2str(j)];
                    logData.Transform{end+1} = mat2str(self.objPlates.initialTargetTransforms{i}(1:3, 4).');



                    self.MoveToPos();

                    if (i == 4 && self.pandaState == 1) || (i == 7 && self.pandaState == 1)
                        if i == 4
                            colour = 'red';
                        elseif i == 7
                            colour = 'blue';
                        end

                        try
                            delete(self.objPlates.stackers{stackCounter})
                            delete(self.plates{i-3});
                            delete(self.plates{i-2});
                            delete(self.plates{i-1});
                        end
                        pos = self.objPlates.plateStack{i - 3}(1:3);
                        pos(3) = pos(3) - 0.04;
                        self.stack{stackCounter} = PlateStacker(transl(pos), colour);
                        self.MoveUR5(stackCounter)
                        stackCounter = stackCounter + 1;
                    end

                    self.pandaState = self.pandaState + 1;
                end
                self.plateCounter = self.plateCounter + 1;
            end

            colour = 'green';
            try
                delete(self.objPlates.stackers{3})
                delete(self.plates{7});
                delete(self.plates{8});
                delete(self.plates{9});
            end
            pos = self.objPlates.plateStack{7}(1:3);
            pos(3) = pos(3) - 0.04;
            self.stack{3} = PlateStacker(transl(pos), colour);

            self.MoveUR5(stackCounter);

            % Display a message indicating the end of the task
            disp('Task Completed Successfully');
            % Log the end time and status
            logData.Time{end+1} = datestr(datetime('now'), 'yyyy-mm-dd HH:MM:SS');
            logData.Status{end+1} = 'Task Completed';
            logData.Transform{end+1} = 'N/A';
            % Save the log data
            save('logData.mat', 'logData');
        end

        function MoveToPos(self)
            self.UpdateRobots();
            steps = 10;
            switch self.pandaState
                case 1
                    disp('CASE 1')
                    targetPos = self.objPlates.safeInitialTargetTransforms{self.plateCounter};
                    rpy = rpy2tr(0, 180, 0, 'deg');
                case 2
                    disp('CASE 2')
                    targetPos = self.objPlates.initialTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 180, 0, 'deg');
                case 3
                    disp('CASE 3')
                    targetPos = self.objPlates.safeInitialTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 180, 0, 'deg');
                case 4
                    disp('CASE 4')
                    targetPos = self.objPlates.safeStackTargetTransforms{self.plateCounter};
                    rpy = rpy2tr(0, 180, 0, 'deg');
                case 5
                    disp('CASE 5')
                    targetPos = self.objPlates.stackTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 90, 180, 'deg');
                case 6
                    disp('CASE 6')
                    targetPos = self.objPlates.safeStackTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 90, 180, 'deg');
            end

            robotXYZ = self.pandaEnd.T;
            robotXYZ = robotXYZ(1:3,4)';
            targetXYZ = targetPos(1:3,4)';

            if self.pandaState == 2 || self.pandaState == 3 || self.pandaState == 5 || self.pandaState == 6
                for i = 1:length(targetXYZ)
                    cartesianPath(:, i) = linspace(robotXYZ(i), targetXYZ(i), steps);
                end
                for i = 1:length(cartesianPath)
                    t = transl(cartesianPath(i,:)) * rpy;
                    pandaAngles = self.panda.model.ikcon(t, self.pandaJointAngles);
                    self.panda.model.animate(pandaAngles);
                    self.UpdateRobots();
                    if self.pandaState == 3 || self.pandaState == 4 || self.pandaState == 5
                        self.MovePlates();
                    end
                    drawnow();
                    % pause(0.1);
                end

            else
                qFinal = self.panda.model.ikcon(targetPos * rpy, self.pandaJointAngles);
                qMatrix = jtraj(self.pandaJointAngles, qFinal, steps);
                self.AnimatePanda(qMatrix);
            end
        end

        function AnimatePanda(self, qMatrix)
            for i = 1:length(qMatrix)
                self.panda.model.animate(qMatrix(i,:));
                self.UpdateRobots();
                if self.pandaState == 3 || self.pandaState == 4 || self.pandaState == 5
                    self.MovePlates();
                end
                drawnow();
                % pause(0.1);
            end
        end

        function MoveUR5(self, stack)
            self.plateStackerModel = self.stack{stack};

            targetTransforms = cell(7);
            targetTransforms{2} = self.plateStackerModel.model.base.T;
            if stack == 1
                targetTransforms{2}(1,4) = targetTransforms{2}(1,4) - self.ur5GriperOffset;
                targetTransforms{3} = targetTransforms{2} * rpy2tr(0, -90, 0, 'deg'); % 0 -90 0 = plates upside down
                targetTransforms{2} = targetTransforms{2} * rpy2tr(0, -90, 90, 'deg');           
                targetTransforms{1} = targetTransforms{2};
                targetTransforms{1}(1,4) = targetTransforms{1}(1,4) - 0.15;
            else
                targetTransforms{2}(2,4) = targetTransforms{2}(2,4) + self.ur5GriperOffset;
                targetTransforms{3} = targetTransforms{2} * rpy2tr(0, -90, 0, 'deg'); % 0 -90 0 = plates upside down
                targetTransforms{2} = targetTransforms{2} * rpy2tr(0, -90, 0, 'deg');           
                targetTransforms{1} = targetTransforms{2};
                targetTransforms{1}(2,4) = targetTransforms{1}(2,4) + 0.15;
            end
            targetTransforms{3}(3,4) = targetTransforms{3}(3,4) + 0.45;
            targetTransforms{5} = self.objPlates.finalTargetTransforms{stack} * rpy2tr(0, -90, 90, 'deg');
            targetTransforms{5}(2,4) = targetTransforms{5}(2,4) + self.ur5GriperOffset;
            targetTransforms{4} = targetTransforms{5};
            targetTransforms{4}(2,4) = targetTransforms{4}(2,4) + 0.25;
            targetTransforms{6} = targetTransforms{4};
            targetTransforms{7} = self.linearUR5.model.fkine(zeros(1,self.linearUR5.model.n));

            % for i = 1:length(targetTransforms)
            %     pos = targetTransforms{i}(1:3,4);
            %     pos = pos';
            %     disp(['pos: ', num2str(i), ' xyz = ', num2str(pos)])
            % end


            for i = 1:length(targetTransforms)
                T1 = targetTransforms{i};

                if i == 1000 % not using RMRC
                    qMatrix = self.ResolvedMotionRateControl('linearUR5', T1)
                else
                    angles = self.linearUR5.model.ikcon(T1, self.ur5JointAngles);
                    qMatrix = jtraj(self.ur5JointAngles, angles, 10);
                end

                for j = 1:size(qMatrix,1)
                    q = qMatrix(j,:)
                    self.linearUR5.model.animate(q);
                    if i >= 3 && i < 6
                        self.MovePlateStacker()
                        drawnow();
                    end
                    drawnow();
                    pause(0.1);

                end
                self.UpdateRobots();
            end
        end

        function UpdateRobots(self)
            self.pandaJointAngles = self.panda.model.getpos();
            self.pandaEnd = self.panda.model.fkine(self.pandaJointAngles);
            self.ur5JointAngles = self.linearUR5.model.getpos();
            self.ur5End = self.linearUR5.model.fkine(self.ur5JointAngles);

            self.gui = GUI(self);
            while self.gui.CheckEstop
                pause(0.5);
                self.UpdateGUI();
            end
            self.gui.UpdateGUI(self);
            if self.AnglesInQLims()
                self.gui.UpdateSliders(self.pandaJointAngles);
            end
        end

        function MovePlates(self)
            self.UpdateRobots();
            platePos = self.pandaEnd.T;

            % Extract the current z-direction of the end effector
            zDirection = platePos(1:3, 3);

            % Compute the offset in the global frame
            globalOffset = zDirection * (self.pandaGriperOffset);

            % Apply the offset to the current position
            platePos(1:3, 4) = platePos(1:3, 4) + globalOffset;

            if self.pandaState == 5
                self.plateModel.model.base = platePos * troty(-pi/2);
            else
                self.plateModel.model.base = platePos * trotx(-pi/2);
            end
            animate(self.plateModel.model, self.plateJoints);
        end

        % Below is the function to check collisions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % function CheckCollisions(self)
        %
        %
        %     for objectCoordinate <
        %         % Stop robot movement
        %         disp("Robot collision occured!! Oh No!!")
        %     end
        %
        % end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % MovePlateStacker function below
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function MovePlateStacker(self)
            self.UpdateRobots();
            plateStackerPos = self.ur5End.T;

            % Extract the current z-direction of the end effector
            zDirection = plateStackerPos(1:3, 3);

            % Compute the offset in the global frame
            % globalOffset = zDirection * (self.ur5GriperOffset)
            globalOffset = zDirection * self.ur5GriperOffset;

            % Apply the offset to the current position
            plateStackerPos(1:3, 4) = plateStackerPos(1:3, 4) + globalOffset;

            % if self.linearUR5 == 1
            %     self.plateStackerModel.model.base = plateStackerPos * troty(-pi/2);
            % else
            %     self.plateStackerModel.model.base = plateStackerPos * trotx(-pi/2);
            % end
            self.plateStackerModel.model.base = plateStackerPos * troty(pi/2);
            animate(self.plateStackerModel.model, self.plateStackerJoints);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function inLimits = AnglesInQLims(self)
            for i = 1:length(self.pandaJointAngles)
                if self.pandaJointAngles(i) < self.panda.model.qlim(i,1) || self.pandaJointAngles(i) > self.panda.model.qlim(i,2)
                    inLimits = false;
                    return;
                end
            end
            inLimits = true;
            return
        end

        function qMatrix = ResolvedMotionRateControl(self, model, tr1)
            if strcmp(model, 'Panda')
                robot = self.panda;
            else 
                robot = self.linearUR5;
            end
            q = robot.model.getpos()
            n = robot.model.n;
            lims = robot.model.qlim;
            velocity = 0.25;
            deltaT = 0.050;  
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1]);  % Weighting matrix for the velocity vector

            
            q0 = robot.model.fkine(q);
            q0 = q0.T;
            startPoint = q0(1:3,4)';
            endPoint = tr1(1:3,4)';
            dist = norm(endPoint - startPoint);
            t = dist/velocity;
            steps = floor(t * 10);

            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,n);       % Array for joint angles
            qdot = zeros(steps,3);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory
            

            % Set up trajectory, initial pose
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            for i=1:steps
                x(1,i) = (1-s(i))*startPoint(1)+s(i)*endPoint(1); % Points in x
                x(2,i) = (1-s(i))*startPoint(2)+s(i)*endPoint(2); % Points in y
                x(3,i) = (1-s(i))*startPoint(3)+s(i)*endPoint(3); % Points in z
                theta(1,i) = 0; % Roll angle
                theta(2,i) = 0; % Pitch angle
                theta(3,i) = 0; % Yaw angle
            end
        
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            qMatrix(1,:) = robot.model.ikcon(T,q);                                     % Solve joint angles to achieve first waypoint
        
            % Track the trajectory with RMRC
            for i = 1:steps-1
                T = robot.model.fkine(qMatrix(i,:));                                    % Get forward transformation at current joint state
                T = T.T;
                deltaX = x(:,i+1) - T(1:3,4);                                       	% Get position error from next waypoint
                linear_velocity = (1/deltaT)*deltaX;
                xdot = W*linear_velocity;                                           	% Calculate end-effector velocity to reach next waypoint.
                J = robot.model.jacob0(qMatrix(i,:));                                   % Get Jacobian at current joint state
                J = J(1:3,1:3);
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1-m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J+lambda*eye(3))*J';                                      % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the vector)
                qlim = robot.model.qlim;
                for j = 1:3                                                             % Loop through joints 1 to 3
                    if J == 3
                        [~, index] = min(abs(lims(:,1)-qMatrix(i,2)));
                        [~, index2] = min(abs(lims(:,3)-qMatrix(i,2)));
                        qlim(3,1) = lims(index,2);
                        qlim(3,2) = lims(index2,4);
                    end 
                    if qMatrix(i,j) + deltaT*qdot(i,j) < qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,1:3) = qMatrix(i,1:3)+deltaT*qdot(i,:);                     % Update next joint state based on joint velocities
            end
            
            qMatrix(:,4) = -qMatrix(:,3);    
        end
    end
end



