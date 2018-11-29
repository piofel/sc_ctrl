classdef LVLH < TransformationMatrices
    %   Class for storing LVLH relative position and velocity
    
    properties (SetAccess = private, GetAccess = public)
        ChiefOrbit
        DeputyOrbit
    end
    
    methods
        function lvlh = LVLH(ChiefOrb, DeputyOrb) % Constructor
            lvlh.ChiefOrbit = ChiefOrb; % ChiefOrb is a KeplerOrbit object
            lvlh.DeputyOrbit = DeputyOrb; % DeputyOrb is a KeplerOrbit object
        end
        function [Rlvlh, Vlvlh] = calcStateVector(lvlh, t) 
            % Calculates LVLH state vector
            % from known chief and deputy orbits
            
            % Position and velocity of the spacecrafts relative to the 
            % geocentric equatorial reference frame
            ChiefCoordinatesGEF = GEF(lvlh.ChiefOrbit);
            DeputyCoordinatesGEF = GEF(lvlh.DeputyOrbit);
            [Rc, Vc] = ChiefCoordinatesGEF.calcStateVector(t);
            [Rd, Vd] = DeputyCoordinatesGEF.calcStateVector(t);          
            
            % Calculate the Q_Xl transformation matrix from
            % the inertial frame into the LVLH frame
            rc = norm(Rc);
            Q_Xl = lvlh.calcTransformationMatrixQ_Xl(Rc, Vc);            
            Omega = cross(Rc, Vc)/rc^2; % The angular velocity of the LVLH frame attached to chief spacecraft

            Rrel = Rd - Rc;
            Vrel = Vd - Vc - cross(Omega, Rrel);            
            Rlvlh = Q_Xl * Rrel'; % Column vectors
            Vlvlh = Q_Xl * Vrel';
        end
    end
    
    methods (Static = true)
        function [rd] = calcRadialDistance(rc, x)
            % Calculates distance between the Earth centre and deputy spacecraft
            % rc - chief spacecraft radial distance
            % x1, x2, x3 - relative position between deputy and chief in LVLH frame
            x1 = x(1);
            x2 = x(2);
            x3 = x(3);
            rd = sqrt((rc+x1)^2 + x2^2 + x3^2);
        end
    end    
end % classdef
