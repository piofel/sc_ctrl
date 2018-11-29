classdef GeneralRelMotModel < IndivStateSpaceModel & CelestialSystem
    %   Class of general model of relative motion
    %   Relative motion model interface
    
    properties (SetAccess = protected, GetAccess = public)
        ChiefOrbit
        t0 % The initial moment of time
        x0 % Initial condition
    end
    
    properties (Constant = true)
    	MeasuredDisturbanceVector = 1; % v
    end
    
    properties (Abstract, Constant = true)
        StateVectorType
    end
    
    methods (Access = protected)
        function Cc = calcContOutputMatrix(~, ~, ~, ~)
            Cc = eye(6);
        end
        function Dc = calcContFeedthroughMatrix(~, ~, ~, ~)
            Dc = zeros(6,3);
        end
    end
end
