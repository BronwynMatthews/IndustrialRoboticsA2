classdef GUIClass
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        objGUI
        Estop = false
    end

    methods
        function self = GUIClass()            
            panda = Panda();
            linearUR5 = LinearUR5custom();
            self.objGUI = GUI(panda, linearUR5)
        end
    end

    methods (Static)
        function GetValues()
        end
    end

end