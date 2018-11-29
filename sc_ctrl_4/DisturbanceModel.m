classdef DisturbanceModel < GeneralMath
    %   Class of input and output disturbance model
    %   Detailed explanation goes here
    
    properties (SetAccess = private, GetAccess = private)
        inDistAmp
        outDistAmp
    end
    
    properties (SetAccess = private, GetAccess = public)
        distTs
    end
    
    methods (Access = public)
        function dmdl = DisturbanceModel(inputDistAmp, outputDistAmp, distSampleTime)
            % Constructor
            dmdl.inDistAmp = inputDistAmp; % m/s^2
            dmdl.outDistAmp = outputDistAmp; % m, m/s
            dmdl.distTs = distSampleTime; % s
        end
        function inDist = calcInputDisturbance(dmdl, SpacecraftMass)
            inDist = zeros(3,1);
            for i=1:3
            	dice = rand;
                if dice >= 0.5
                    s = 1;
                else
                    s = -1;
                end                
                inDist(i) = s * dmdl.inDistAmp * SpacecraftMass * rand; % m/s^2 
            end
        end
        function outDist = calcOutputDisturbance(dmdl)
            outDist = zeros(7,1);
            for i=1:7
                dice = rand;
                if dice >= 0.5
                    s = 1;
                else
                    s = -1;
                end   
                outDist(i) = s * dmdl.outDistAmp * rand; % m, m/s
            end
        end
    end
    
end

