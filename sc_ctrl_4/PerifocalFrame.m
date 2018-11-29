classdef PerifocalFrame < CelestialSystem
    % Class for storing Perifocal Frame data and its methods
    % Detailed explanation goes here
    
    properties (SetAccess = private, GetAccess = public)
        KepOrb % Keplerian orbit
    end
    
    methods
        function perifocal = PerifocalFrame(kepOrb) % Constructor
            perifocal.KepOrb = kepOrb; % kepOrb is a KeplerOrbit object
        end
        function [rp, vp] = calcStateVector(perifocal, t)
            % Computes position vector in perifocal coordinates
            mu = perifocal.GrPrm;
            h = perifocal.KepOrb.h;
            e = perifocal.KepOrb.coe.e;
            TA = perifocal.KepOrb.calcTrueAnomaly(t);
            rp = (h^2/mu) * (1/(1 + e*cos(TA))) * (cos(TA)*[1;0;0] ...
                 + sin(TA)*[0;1;0]);
            vp = (mu/h) * (-sin(TA)*[1;0;0] + (e + cos(TA))*[0;1;0]);
        end
    end    
end

