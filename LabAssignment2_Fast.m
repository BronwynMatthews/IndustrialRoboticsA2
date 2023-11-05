classdef LabAssignment2_Fast < handle
    properties
        % GUI
        guiObj;
        startup = true;
        
        % LIGHTS
        blueLight1;
        blueLight2;
        flash = true;
        greenLight1;
        greenLight2;
        redLight1;
        redLight2;

        % LOG TABLE
        logTable;
        
        % PERSON
        person;
        personPoint1;
        personPoint2;
        personPos;
        
        % PLATES
        objPlates;
        plateCounter = 1;
        plateJoints = [0,0];
        plateModel;
        plateStackerJoints = [0,0];
        plateStackerModel;
        plates;
        stack;
    
        % ROBOTS
        gripper1;
        gripper2;
        linearUR5;
        panda;
        pandaEnd;
        pandaGripperOffset = 0.12; % RADIUS OF A PLATE
        pandaJointAngles;
        pandaState;
        robotRunning = 'Panda';
        ur5End;
        ur5GripperOffset = 0.25; % OFFSET FOR END EFFECTOR/PLATES
        ur5JointAngles;
    
        % SAFETY
        collisionRectangles;
        lightCurtains;
        rectPrismData;

        % HARDWARE
        hardware = false; 
    end

    methods
        function self = LabAssignment2_Fast()
            cla;
            % clf;
            clc;

            h = findall(0, 'Type', 'figure', 'Name', 'MATLAB App');
            if ~isempty(h)
                close(h);
            end

            hold on;
            
            self.InitialiseRobots();
            self.guiObj = GUI(self.linearUR5, self.panda, self.hardware, 'COM3'); % Creates a GUI model and passes in the two robot models as well as a bool for whether the system runs with hardware or not.
            self.objPlates = InitialisePlates(); % Calculates the positions/tranforms of all the plates in start, stack, and final location
            self.InitialiseEnvironment();

            self.RunRobots();
        end

        function InitialiseRobots(self)
            self.panda = Panda(transl(1.6, 3.0, 0.95));
            self.linearUR5 = LinearUR5custom(transl(0.4, 2.6, 0.95));

            self.UpdateRobots();

            self.gripper1 = PandaGripper(self.pandaEnd, 1);
            self.gripper2 = PandaGripper(self.pandaEnd, 2);
        end

        function InitialiseEnvironment(self)
            Environment(); % BUILD THE WORKSPACE
            self.personPoint1 = [2.5, 0, 0.5];
            self.person = Person(transl(2.5, 0, 0)); % PLACE A PERSON IN THE WORKSPACE
             % STORE THE START LOCATION OF THE PERSON FOR LINE PLANE INTERSECTION

            self.lightCurtains = LightCurtains(); % CALCULATE THE PLANES OF EACH LIGHT CURTAIN

            self.collisionRectangles = {
                struct('lower', [-2.4, 3.27, 0], 'upper', [3.9, 3.45, 2.1]) % Wall
                struct('lower', [-1.03, 1.5, 1.38], 'upper', [1.12, 2.15, 1.4]) % Plate stacker bench location
                struct('lower', [-1.7, 1, 0.8], 'upper', [1.7, 3.25, 0.9]) % Bench both robots mounted too
                struct('lower', [-3.15, 2.4, 0], 'upper', [-1.65, 3.15, 1.95]) % Fridge
            }; % RECTANGLES FOR COLLISION DETECTION

            self.rectPrismData = cell(length(self.collisionRectangles),3);
            
            for i = 1:length(self.collisionRectangles)
                [tempVertex, tempFace, tempFaceNormals] = RectangularPrism(self.collisionRectangles{i}.lower, self.collisionRectangles{i}.upper);
                self.rectPrismData{i,1} = tempVertex;
                self.rectPrismData{i,2} = tempFace;
                self.rectPrismData{i,3} = tempFaceNormals;
            end

            self.stack = cell(3,1);

            % THE BELOW FOR LOOP PLOTS ALL THE PLATES IN THE WORKSPACE
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
        
        function CheckGUI(self) % UPDATES THE VALUES OF THE GUI - CALLED RECURSIVELY FROM THE UPDATEROBOTS() FUNCTION
            self.guiObj.UpdateGUI;
            % self.FlashLights(); % commented out for 'fast' simulation
            while ~self.guiObj.running
                if ~self.guiObj.estop
                    [tempQMatrix, robot] = self.guiObj.GetSliderValues();
                    robot.model.animate(tempQMatrix);
                end
                % self.FlashLights();  % commented out for 'fast' simulation
                % self.guiObj.JogRobot();
                pause(0.1);
                self.guiObj.UpdateGUI;
            end           
        end

        function RunRobots(self)
            self.UpdateRobots();
            stackCounter = 1;

            % Initialise an empty table with the necessary variables
            self.logTable = table(datetime, string, string, {zeros(4)}, {zeros(4)}, ...
                'VariableNames', {'Time', 'Status', 'Robot', 'Target_Transform', 'Reached_Transform'});
            
            % Fill in a row of the table
            newRow = {datetime('now','Format', 'dd/MM/uuuu HH:mm:ss'), 'Task Started', 'Panda', self.pandaEnd.T, self.pandaEnd.T};
            self.logTable = [self.logTable; newRow];

            newRow = {datetime('now','Format', 'dd/MM/uuuu HH:mm:ss'), 'Task Started', 'LinearUR5', self.ur5End.T, self.ur5End.T};
            self.logTable = [self.logTable; newRow];

            for i = 1:self.objPlates.numOfPlates
                disp(['Panda unstacking plate ', num2str(i)]);
                self.plateModel = self.plates{i}; % DEFINES THE PLATE TO BE ANIMATED
                self.plateCounter = i; % GLOBAL VARIABLE FOR PLATE COUNT

                for j = 1:6 % MOVES THE PANDA THROUGH EACH TRANSFORM FOR DISHWASHER UNSTACKING
                    self.pandaState = j;

                    self.MovePanda();

                    self.logTable.Reached_Transform{end} = self.pandaEnd.T;

                    if self.pandaState == 1 && (i == 4 || i == 7)
                        self.MoveUR5(stackCounter)

                        stackCounter = stackCounter + 1;
                    end
                end
            end

            self.plateCounter = self.plateCounter + 1;
            
            newRow = {datetime('now','Format', 'dd/MM/uuuu HH:mm:ss'), 'All tasks completed successfully!', self.robotRunning, self.pandaEnd.T, self.pandaEnd.T};
            self.logTable = [self.logTable; newRow];

            self.MoveUR5(stackCounter)

            newRow = {datetime('now','Format', 'dd/MM/uuuu HH:mm:ss'), 'All tasks completed successfully!', self.robotRunning, self.ur5End.T, self.ur5End.T};
            self.logTable = [self.logTable; newRow];

            % Display a message indicating the end of the task
            disp('All Tasks Completed Successfully');

            logTable = self.logTable;

            % Save the log data
            save('logTable.mat', 'logTable');
        end

        function MovePanda(self)
            self.UpdateRobots();
            steps = 10;
            self.robotRunning = 'Panda';
            
            switch self.pandaState
                case 1
                    disp('CASE 1')
                    targetPos = self.objPlates.safeInitialTargetTransforms{self.plateCounter};
                    rpy = rpy2tr(0, 180, 0, 'deg');
                    state = 'Moving to safe position above initial plate position (WITHOUT plate)';
                case 2
                    disp('CASE 2')
                    targetPos = self.objPlates.initialTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 180, 0, 'deg');
                    state = 'Picking up plate from initial plate position';
                case 3
                    disp('CASE 3')
                    targetPos = self.objPlates.safeInitialTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(0, 180, 0, 'deg');
                    state = 'Moving back to safe position above initial plate position (WITH plate)';
                case 4
                    disp('CASE 4')
                    targetPos = self.objPlates.safeStackTargetTransforms{self.plateCounter};
                    rpy = rpy2tr(-90, 180, 90, 'deg');
                    state = 'Moving to safe position above final plate position (WITH plate)';
                case 5
                    disp('CASE 5')
                    targetPos = self.objPlates.stackTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(-90, 180, 90, 'deg');
                    state = 'Placing plate in its final position';
                case 6
                    disp('CASE 6')
                    targetPos = self.objPlates.safeStackTargetTransforms{self.plateCounter};
                    steps = 10;
                    rpy = rpy2tr(-90, 180, 90, 'deg');
                    state = 'Moving back to safe position above final plate position (WITHOUT plate)';
            end

            disp(state);

            targetPos = targetPos * rpy;

            newRow = {datetime('now','Format', 'dd/MM/uuuu HH:mm:ss'), state, self.robotRunning, targetPos, {}};
            self.logTable = [self.logTable; newRow];

            if self.pandaState == 1 || self.pandaState == 4 || self.pandaState == 6 
                qFinal = self.panda.model.ikcon(targetPos, self.pandaJointAngles);
                qMatrix = jtraj(self.pandaJointAngles, qFinal, steps);
            else
                qMatrix = ResolvedMotionRateControl(self.panda, targetPos, 'vertical');
            end
            self.AnimatePanda(qMatrix);
        end

        function AnimatePanda(self, qMatrix)
            for i = 1:length(qMatrix)
                self.panda.model.animate(qMatrix(i,:));
                self.UpdateRobots();
                if self.pandaState == 3 || self.pandaState == 4 || self.pandaState == 5
                    self.MovePlates();
                    self.AnimateGripper('close')
                else
                    self.AnimateGripper('open')
                end
                drawnow();
            end
        end

        function MoveUR5(self, stackCounter)
            self.robotRunning = 'LinearUR5';
            if stackCounter == 1
                colour = 'red';
            elseif stackCounter == 2
                colour = 'blue';
            elseif stackCounter == 3
                colour = 'green';
            end
            
            pos = self.objPlates.plateStack{1 + ((stackCounter - 1) * 3)}(1:3);
            pos(3) = pos(3) - 0.04;
            self.stack{stackCounter} = PlateStacker(transl(pos), colour);
            
            self.plateStackerModel = self.stack{stackCounter};
            try
                delete(self.objPlates.stackers{stackCounter})
                delete(self.plates{self.plateCounter - 3});
                delete(self.plates{self.plateCounter - 2});
                delete(self.plates{self.plateCounter - 1});
            end

            targetTransforms = self.objPlates.PlateStackerTransforms(self.linearUR5, stackCounter, self.plateStackerModel);

            for i = 1:7
                tr = targetTransforms{i};

                status = ['Moving stack ', num2str(stackCounter), ' to shelf'];

                newRow = {datetime('now','Format', 'dd/MM/uuuu HH:mm:ss'), status, self.robotRunning, tr, {}};
                self.logTable = [self.logTable; newRow];

                if i == 4 % using RMRC
                    qMatrix = ResolvedMotionRateControl(self.linearUR5, tr, 'horizontal');
                else
                    angles = self.linearUR5.model.ikcon(tr, self.ur5JointAngles);
                    qMatrix = jtraj(self.ur5JointAngles, angles, 10);
                end

                for j = 1:size(qMatrix,1)
                    q = qMatrix(j,:);
                    self.linearUR5.model.animate(q);
                    if i >= 3 && i < 6
                        self.MovePlateStacker()
                    end
                    drawnow();
                end
                self.UpdateRobots();

                self.logTable.Reached_Transform{end} = self.ur5End.T;
            end
        end

        function UpdateRobots(self)
            self.pandaJointAngles = self.panda.model.getpos();
            self.pandaEnd = self.panda.model.fkine(self.pandaJointAngles);
            self.ur5JointAngles = self.linearUR5.model.getpos();
            self.ur5End = self.linearUR5.model.fkine(self.ur5JointAngles);

            if ~self.startup()
                self.CheckGUI();

                if isvalid(self.person)
                    tr = self.person.model.base;
                    tr = tr.T;
                    self.personPos = tr(1:3,4)';
                    self.personPoint2 = self.personPos;
                    self.personPoint2(3) = self.personPoint2(3) + 0.5;
                    self.personPoint2(2) = self.personPoint2(2) + 0.15;
                    self.personPos(2) = self.personPos(2) + 0.05;
                    self.person.model.base = transl(self.personPos) * trotz(pi/2);
                    self.person.model.animate(0);
                    drawnow();
                    
                    for i = 1:3
                        [intersectionPoint, check] = LinePlaneIntersection(self.lightCurtains.normals(i,:), self.lightCurtains.midPoints(i,:), self.personPoint1, self.personPoint2);
                        if check == 1
                            if intersectionPoint(1) > self.lightCurtains.xLims(i,1) && intersectionPoint(1) < self.lightCurtains.xLims(i,2)
                                if intersectionPoint(2) > self.lightCurtains.yLims(i,1) && intersectionPoint(2) < self.lightCurtains.yLims(i,2)
                                    if intersectionPoint(3) > self.lightCurtains.zLims(i,1) && intersectionPoint(3) < self.lightCurtains.zLims(i,2)

                                        disp(['Collision with Light Curtain ', num2str(i)])
                                        self.guiObj.estop = true;
                                        try
                                            delete(self.person);
                                        end
                                    end
                                end
                            end
                        end
                    end
                end

                if strcmp('Panda',self.robotRunning) 
                    if IsCollision(self.panda, self.pandaJointAngles, self.rectPrismData)
                        disp('Collision detected!');
                        self.guiObj.estop = true;
                    end
                else
                    if IsCollision(self.linearUR5, self.ur5JointAngles, self.rectPrismData )
                        disp('Collision detected!');
                        self.guiObj.estop = true;
                    end
                end
            end
            self.startup = false;    
        end

        function FlashLights(self) % NOT USED FOR SIMULATION EFFICIENCY
            if self.flash
                    if self.guiObj.estop 
                        %RED
                        self.redLight1 = PlaceObject('LightOnRed_fridge.ply', [-1.7, 3, 1.6]);
                        self.redLight2 = PlaceObject('LightOnRed_backWall.ply', [2.1, 3.25, 1.6]);
                        
                    elseif self.guiObj.running
                        %GREEN
                        self.greenLight1 = PlaceObject('LightOnGreen_fridge.ply', [-1.7, 3, 1.6]);
                        self.greenLight2 = PlaceObject('LightOnGreen_backWall.ply', [2.1, 3.25, 1.6]);
                    else
                        %BLUE
                        self.blueLight1 = PlaceObject('LightOnBlue_fridge.ply', [-1.7, 3, 1.6]);
                        self.blueLight2 = PlaceObject('LightOnBlue_backWall.ply', [2.1, 3.25, 1.6]);
                    end
                else
                    try
                        delete(self.redLight1);
                        delete(self.redLight2);
                        delete(self.greenLight1);
                        delete(self.greenLight2);
                        delete(self.blueLight1);
                        delete(self.blueLight2);                       
                    end
            end
            self.flash = ~self.flash;
        end

        function MovePlates(self)
            self.UpdateRobots();
            platePos = self.pandaEnd.T;

            % Extract the current z-direction of the end effector
            zDirection = platePos(1:3, 3);

            % Compute the offset in the global frame
            globalOffset = zDirection * (self.pandaGripperOffset);

            % Apply the offset to the current position
            platePos(1:3, 4) = platePos(1:3, 4) + globalOffset;

            self.plateModel.model.base = platePos * trotx(-pi/2);
            animate(self.plateModel.model, self.plateJoints);
        end

        function MovePlateStacker(self)
            self.UpdateRobots();
            plateStackerPos = self.ur5End.T;

            % Extract the current z-direction of the end effector
            zDirection = plateStackerPos(1:3, 3);

            % Compute the offset in the global frame
            globalOffset = zDirection * self.ur5GripperOffset;

            % Apply the offset to the current position
            plateStackerPos(1:3, 4) = plateStackerPos(1:3, 4) + globalOffset;

            self.plateStackerModel.model.base = plateStackerPos * troty(pi/2);
            animate(self.plateStackerModel.model, self.plateStackerJoints);
        end
        
        function AnimateGripper(self, state)
            self.gripper1.model.base = self.gripper1.model.base.T;
            self.gripper1.model.base = self.pandaEnd.T * trotx(pi);
            self.gripper2.model.base = self.gripper1.model.base;

            self.gripper1.model.base = self.gripper1.model.base.T * troty(-pi/2) * trotz(pi) * trotx(-pi/2);
            self.gripper2.model.base = self.gripper2.model.base.T * troty(pi/2) * trotx(-pi/2);
 
            if strcmp(state, 'open')
                self.gripper1.model.animate(0.03)
                self.gripper2.model.animate(0.03)
            else
                self.gripper1.model.animate(0.005)
                self.gripper2.model.animate(0.005)
            end
        end
    end
end



