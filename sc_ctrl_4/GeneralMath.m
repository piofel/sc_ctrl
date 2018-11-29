classdef GeneralMath < handle
    %   Class for storing general math methods and constants
    
    properties (Constant = true)
    end
    
    methods
        function angle = formatAngle(~, angle0) % Format angle so that it is between 0 and 2*pi
            while angle0 > (2*pi)
                angle0 = angle0 - 2*pi;
            end
            while angle0 < 0
                angle0 = angle0 + 2*pi;
            end
            angle = angle0;
        end
        function [A, B, C, D] = calcJacobian(~, F, G, x, u)
            A.symbolic = jacobian(F, x);
            B.symbolic = jacobian(F, u);
            C.symbolic = jacobian(G, x);
            D.symbolic = jacobian(G, u);
        end
    end    
end
