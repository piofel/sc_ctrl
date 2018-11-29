classdef LvlhRelMotModel < GeneralRelMotModel
    %   Class for LVLH family models
    %   Detailed explanation goes here
    
    properties (Constant = true)
        StateVectorType = 'LVLH';
        % In this model x is a set of relative position and velocity       
    end
    
    methods (Access = protected)
        function setInitCond(model, initCond, initCondType)
             if strcmp(initCondType, 'Direct')
                model.x0 = initCond; % Direct initialization of relative motion state
                                     % In this case the initCond is initial
                                     % relative motion state
                                     % (column vector)
             elseif strcmp(initCondType, 'DeputyInitOrbit')
                deputyInitialOrbit = initCond;  % In this case initCond is KeplerOrbit
                                                % object (deputy initial orbit)
                chiefOrbit = model.ChiefOrbit;
                LvlhFrame = LVLH(chiefOrbit, deputyInitialOrbit);
                [Rlvlh, Vlvlh] = LvlhFrame.calcStateVector(model.t0);
                % keyboard
                model.x0 = [Rlvlh; Vlvlh]; % Relative motion state at t0 (column vector)
             end
        end
        function Bc = calcContControlMatrix(~, md, ~, ~, ~)
            Bc = [0        0       0
                  0        0       0
                  0        0       0
                  1/md     0       0
                  0        1/md	   0
                  0        0       1/md];
        end
    end    
end
