classdef TransformationMatrices < handle
    %	Class for storing of transformation matrices
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function Q_pX = calcTransformationMatrixQ_pX(gef) 
            % Calculates the Q_pX transformation matrix from perifocal to geocentric equatorial coordinates            
            RA = gef.KepOrb.coe.RA;
            incl = gef.KepOrb.coe.incl;
            w = gef.KepOrb.coe.w;

            R3_W = [ cos(RA) sin(RA) 0
                    -sin(RA) cos(RA) 0
                     0       0       1];

            R1_i = [1 0          0
                    0 cos(incl)  sin(incl)
                    0 -sin(incl) cos(incl)];

            R3_w = [ cos(w) sin(w) 0
                    -sin(w) cos(w) 0
                     0      0      1];

            Q_pX = R3_W'*R1_i'*R3_w';
        end
        function Q_Xl = calcTransformationMatrixQ_Xl(~, Rc, Vc)
            % Calculates the Q_Xl transformation matrix from
            % the GEF inertial frame into the LVLH frame
            % Rc, Vc - chief object position and velocity vectors in
            % geocentric equatorial frame (GEF)
            rc = norm(Rc);
            I = Rc/rc;
            Hc = cross(Rc, Vc); % Massless angular momentum vector of the chief object
            hc = norm(Hc);
            K = Hc/hc;
            J = cross(K,I);
            Q_Xl = [I; J; K]; % Transformation matrix
        end
        function Q_lX = calcTransformationMatrixQ_lX(gef, Rc, Vc)
            % Calculates the Q_lX transformation matrix from LVLH frame
            % to the GEF inertial frame
            % Enables calculation of deputy GEF coordinates using chief
            % coordinates and LVLH vector
            Q_Xl = gef.calcTransformationMatrixQ_Xl(Rc, Vc);
            Q_lX = inv(Q_Xl);
        end
    end    
end

