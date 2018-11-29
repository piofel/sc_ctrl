classdef GeneralController < GeneralMath
    %   Class for storing general controller properties and methods
    %   Detailed explanation goes here
    
    properties (GetAccess = public, SetAccess = protected)
        Ts
        Reference
    end
        
    methods (Abstract)
        calcControllerMove(~)
    end    
end
