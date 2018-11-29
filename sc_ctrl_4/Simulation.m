classdef Simulation < handle
    %	Class for storing simulation data and methods
    %   Detailed explanation goes here
    
    properties (SetAccess = private, GetAccess = public)
        StartTime % The initial moment of time t0
        RelMotModel % Relative motion model
        MassModel % Model of deputy spacecraft mass
        DistModel % Disturbance model
        Controller % Controller object
        Ts % Sampling time
    end
    
    methods
        function simul = Simulation(startTime, relMotMdl, massMdl, distModel, controller, Tsampl) % Constructor
            simul.StartTime = startTime;
            simul.RelMotModel = relMotMdl;
            simul.MassModel = massMdl;
            simul.DistModel = distModel;
            simul.Controller = controller;
            simul.Ts = Tsampl;
        end
    end    
end
