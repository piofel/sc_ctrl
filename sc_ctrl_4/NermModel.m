classdef NermModel < LvlhRelMotModel
    %   Class for Nonlinear Equations of Relative Motion model
    %   Detailed explanation goes here

    methods
        function nerm = NermModel(chiefOrbit, initCond, initCondType, startTime)
            % Constructor
            if nargin ~= 0
                nerm.ChiefOrbit = chiefOrbit; % ChiefOrb is a KeplerOrbit object
                nerm.t0 = startTime;
                nerm.setInitCond(initCond, initCondType);
            end
        end
        function dx = calcDerivatives(nerm, md, x, u, t)
            Ac = nerm.calcContStateMatrix(x, u, t);
            Bc = nerm.calcContControlMatrix(md, x, u, t);
            Rc = nerm.calcContResidualMatrix(x, u, t);
            dx = Ac*x + Bc*u + Rc;
        end
        function xkp1 = calcNextState(nerm, md, x, u, t, Ts)
            Ad = nerm.calcDiscStateMatrix(x, u, t, Ts);
            Bd = nerm.calcDiscControlMatrix(md, x, u, t, Ts);
            Rd = nerm.calcDiscResidualMatrix(x, u, t, Ts);
            xkp1 = Ad * x + Bd * u + Rd;
        end
        function [Mvar, df, rc, drc, gp, rd] = calcModelVariables(nerm, x, ~, t)
            df = nerm.ChiefOrbit.calcTrueAnomalyRate(t); % True anomaly rate of the chief object
            rc = nerm.ChiefOrbit.calcRadialDistance(t);
            drc = nerm.ChiefOrbit.calcRadialVelocity(t);
            gp = nerm.GrPrm;
            rd = LVLH.calcRadialDistance(rc, x); % Distance between the Earth centre and deputy spacecraft
            Mvar = [df, rc, drc, gp, rd];
        end
    end
    
    methods (Access = protected)
        function Ac = calcContStateMatrix(nerm, x, u, t)
            [~, df, rc, drc, gp, rd] = nerm.calcModelVariables(x, u, t);
            
            Ac41 = df^2 - gp/rd^3;
            Ac42 = -2 * df * drc/rc;
            Ac45 = 2 * df;
            Ac51 = 2 * df * drc/rc;
            Ac52 = df^2 - gp/rd^3;
            Ac54 = -2 * df;
            Ac63 = -gp/rd^3;

            Ac = [0       0       0       1       0       0;
                  0       0       0       0       1       0;
                  0       0       0       0       0       1;
                  Ac41    Ac42    0       0       Ac45    0;
                  Ac51    Ac52    0       Ac54    0       0;
                  0       0       Ac63    0       0       0];
        end
    end
    methods (Access = private)
        function Rc = calcContResidualMatrix(nerm, x, ~, t)
            mu = nerm.GrPrm;
            rc = nerm.ChiefOrbit.calcRadialDistance(t);
            rd = LVLH.calcRadialDistance(rc, x);
            Rc = [0;
                  0;
                  0;
                  mu * (1/rc^2 - rc/rd^3);
                  0;
                  0];
        end
        function Rd = calcDiscResidualMatrix(nerm, x, u, t, Ts)
            Rc = nerm.calcContResidualMatrix(x, u, t);
            Rd = Ts * Rc;
        end
    end
end
