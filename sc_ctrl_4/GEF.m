classdef GEF < TransformationMatrices
    % Class for storing Geocentric Equatorial Frame data and its info
    % Detailed explanation goes here
    
    properties
        KepOrb % Keplerian orbit
    end
    
    methods
        function gef = GEF(kepOrb) % Constructor
            gef.KepOrb = kepOrb; % kepOrb is a KeplerOrbit object
        end
        function [R, V] = calcStateVector(gef, t)
            % Calculates the position R and velocity V vectors in the geocentric equatorial frame of reference
            perifocal = PerifocalFrame(gef.KepOrb);
            [Rp, Vp] = perifocal.calcStateVector(t);
            Q_pX = gef.calcTransformationMatrixQ_pX();
            R = Q_pX * Rp; 
            V = Q_pX * Vp;
            R = R'; % Now R and V are row vectors
            V = V';
        end
        function [Rd, Vd] = calcDeputyStateVectorFromLVLH(ChiefGEF, LvlhStateVector, t)
            % Calculates the deputy position Rd and velocity Vd vectors in the geocentric equatorial frame
            % using chief state vector and LVLH state vector
            [Rc, Vc] = ChiefGEF.calcStateVector(t); % Chief satellite state vector in GEF
            rc = norm(Rc);
            Rlvlh = [LvlhStateVector(1), LvlhStateVector(2), LvlhStateVector(3)];
            Vlvlh = [LvlhStateVector(4), LvlhStateVector(5), LvlhStateVector(6)];
            Q_lX = ChiefGEF.calcTransformationMatrixQ_lX(Rc, Vc);
            Omega = cross(Rc, Vc)/rc^2; % The angular velocity of the LVLH frame attached to chief spacecraft
            
            Rrel = Q_lX * Rlvlh';
            Vrel = Q_lX * Vlvlh';
            Rrel = Rrel';
            Vrel = Vrel';
            Rd = Rc + Rrel;
            Vd = Vc + cross(Omega, Rrel) + Vrel;
        end
    end  
end

