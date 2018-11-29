classdef ThModel < LvlhRelMotModel
    %   Class for Tschauner-Hempel Model of Relative Motion
    %   Detailed explanation goes here

    methods
        function thm = ThModel(chiefOrbit, initCond, initCondType, startTime)
            % Constructor
            if nargin ~= 0
                thm.ChiefOrbit = chiefOrbit; % ChiefOrb is a KeplerOrbit object
                thm.t0 = startTime;
                thm.setInitCond(initCond, initCondType);
            end
        end
        function dx = calcDerivatives(thm, md, x, u, t)
            Ac = thm.calcContStateMatrix(x, u, t);
            Bc = thm.calcContControlMatrix(md, x, u, t);
            Rc = thm.calcContResidualMatrix(x, u, t);
            dx = Ac*x + Bc*u + Rc;
        end
        function xkp1 = calcNextState(thm, md, x, u, t, Ts)
            Ad = thm.calcDiscStateMatrix(x, u, t, Ts);
            Bd = thm.calcDiscControlMatrix(md, x, u, t, Ts);
            Rd = thm.calcDiscResidualMatrix(x, u, t, Ts);
            xkp1 = Ad * x + Bd * u + Rd;
        end
        function [Mvar, df, rc, drc, gp, rd] = calcModelVariables(thm, x, ~, t)
            df = thm.ChiefOrbit.calcTrueAnomalyRate(t); % True anomaly rate of the chief object
            rc = thm.ChiefOrbit.calcRadialDistance(t);
            drc = thm.ChiefOrbit.calcRadialVelocity(t);
            gp = thm.GrPrm;
            rd = LVLH.calcRadialDistance(rc, x); % Distance between the Earth centre and deputy spacecraft
            Mvar = [df, rc, drc, gp, rd];
        end
        function Rc = calcContNonlinearElement(thm, ~, x, u, t)
            Rc = thm.calcContResidualMatrix(x, u, t);
        end
        function Rd = calcDiscNonlinearElement(thm, ~, x, u, t, Ts)
            Rd = thm.calcDiscResidualMatrix(x, u, t, Ts);
        end
    end
    methods (Access = protected)
        function Ac = calcContStateMatrix(thm, x, u, t)
            [~, df, rc, drc, gp, ~] = thm.calcModelVariables(x, u, t);
            
            Ac41 = df^2 + 2*gp/rc^3;
            Ac42 = -2 * df * drc/rc;
            Ac45 = 2 * df;
            Ac51 = 2 * df * drc/rc;
            Ac52 = df^2 - gp/rc^3;
            Ac54 = -2 * df;
            Ac63 = -gp/rc^3;

            Ac = [0       0       0       1       0       0;
                  0       0       0       0       1       0;
                  0       0       0       0       0       1;
                  Ac41    Ac42    0       0       Ac45    0;
                  Ac51    Ac52    0       Ac54    0       0;
                  0       0       Ac63    0       0       0];
        end
    end
    methods (Access = private)
        function Rc = calcContResidualMatrix(~, ~, ~, ~)
            Rc = [0;
                  0;
                  0;
                  0;
                  0;
                  0];
        end
        function Rd = calcDiscResidualMatrix(thm, x, u, t, Ts)
            Rc = thm.calcContResidualMatrix(x, u, t);
            Rd = Ts * Rc;
        end
    end
end
