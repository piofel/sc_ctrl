classdef MPControllerParameters < handle
    %   Class for storing model predictive controller parameters
    %   Detailed explanation goes here
    
    properties (GetAccess = public, SetAccess = public)
        PredictionHorizon
        ControlHorizon
        
        ControlConstraint
        ControlRateConstraint
        OutputVariablesConstraint
        
        ControlWeight
        ControlRateWeight
        OutputVariablesWeight
    end
    
    methods
        function mpcParam = MPControllerParameters(p, m, cc, crc, ovc, cw, crw, ovw)
            mpcParam.PredictionHorizon = p;
            mpcParam.ControlHorizon = m;
            
            mpcParam.ControlConstraint = cc;
            mpcParam.ControlRateConstraint = crc;
            mpcParam.OutputVariablesConstraint = ovc;
            
            mpcParam.ControlWeight = cw;
            mpcParam.ControlRateWeight = crw;
            mpcParam.OutputVariablesWeight = ovw;
        end
    end
end
