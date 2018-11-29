classdef HcwModel < LvlhRelMotModel
    %   Class for Hill–Clohessy–Wiltshire model
    %   Detailed explanation goes here

    methods
        function hcw = HcwModel(chiefOrbit, initCond, initCondType, startTime) % Constructor
            hcw.ChiefOrbit = chiefOrbit; % ChiefOrb is a KeplerOrbit object
            hcw.t0 = startTime;
            hcw.setInitCond(initCond, initCondType);
        end
    end
    methods (Access = protected)
        function Ac = calcContStateMatrix(hcw, ~, ~, ~)
            n = hcw.ChiefOrbit.n;
            Ac = [0           0       0           1       0       0;
                  0           0       0           0       1       0;
                  0           0       0           0       0       1;
                  3*(n^2)     0       0           0       2*n     0;
                  0           0       0           -2*n    0       0;
                  0           0       -(n^2)      0       0       0];
        end
    end    
end
