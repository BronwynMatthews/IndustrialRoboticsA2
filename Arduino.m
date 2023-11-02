estopActivated = false;
programStarted = false;

while true
    if arduinoObj.AvailableBytes > 0
        data = readline(arduinoObj);
        disp(data); % Display received data for debugging
        
        if contains(data, "E-STOP")
            if estopActivated
                disp("E-STOP Deactivated");
                estopActivated = false;
            else
                disp("E-STOP Activated");
                estopActivated = true;
                programStarted = false; % Ensure program does not run when E-STOP is active
            end
        elseif contains(data, "START")
            if ~estopActivated
                if programStarted
                    disp("Program Stopped");
                    programStarted = false;
                else
                    disp("Program Started");
                    programStarted = true;
                end
            end
        end
        
        % Handle your robotics tasks here based on the state of `estopActivated` and `programStarted`
        
    end
    
    pause(0.1); % Avoid overwhelming the MATLAB loop with too-fast iterations
end
