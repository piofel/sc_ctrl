classdef COE < handle
    %   Class for storing Classical Orbital Elements data and its info
    %   Detailed explanation goes here
    
    properties (SetAccess = private, GetAccess = public)
        a  % Semi-major axis 
        e  % Eccentricity
        incl  % Inclination 
        RA % Right ascension of the ascending node
        w  % Argument of periapsis
        M0  % Mean anomaly at specific time moment t0
        t0  % The value of specific time moment (when M = M0)
        Units % km & deg (0) or m & rad (1)
    end
    
    methods
        function coe = COE(coeVector, coeInfo, coeUnits) % Constructor
            coe.a = coeVector(1);
            coe.e = coeVector(2);
            coe.incl = coeVector(3);
            coe.RA = coeVector(4);
            coe.w = coeVector(5);
            coe.M0 = coeVector(6);
            coe.t0 = coeInfo(1); % The value of specific time moment t0 (when M = M0)
            coe.Units = coeUnits(1); % km & deg (0) or m & rad (1)
            coe.formatToMeterRad
        end
        function formatToMeterRad(coe) % Converts units from km & deg to m & rad
            if coe.Units == 0 % If COE units are km & deg
               coe.a = coe.a * 1000; % a in meters
               coe.incl = coe.incl * pi/180; % incl in radians
               coe.RA = coe.RA * pi/180; % RA in radians
               coe.w = coe.w * pi/180; % w in radians
               coe.M0 = coe.M0 * pi/180; % M0 in radians
               coe.Units = 1; % Now COE data format is m & rad
            end
        end
        function [CoeVector, CoeVectorInfo] = retrieveVector(coe)
            CoeVector = [coe.a coe.e coe.incl coe.RA coe.w coe.M0]';
            CoeVectorInfo = [coe.Units coe.t0]';
        end
    end
end % classdef    
