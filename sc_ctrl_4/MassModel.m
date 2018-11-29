classdef MassModel < IndivStateSpaceModel & CelestialSystem
    % Class for model of deputy spacecraft mass
    % Detailed explanation goes here
    
    properties (SetAccess = private, GetAccess = public)
        mdry % Mass of spacecraft without fuel
        mf0 % Mass of the fuel at the initial moment
        si % Specific impulse of the thrusters
        expf0 % Amount of expelled fuel at the initial time t0
    end
    
    methods
        function massMdl = MassModel(mDry, mF0, sImp, expFuel0) % Constructor
            massMdl.mdry = mDry;
            massMdl.mf0 = mF0;
            massMdl.si = sImp;
            massMdl.expf0 = expFuel0;
        end
        function yc = calcContOutputDeputyMass(massMdl, x, u, t)
            % Output of total deputy mass
            Cc = massMdl.calcContOutputMatrixDeputyMass(x, u, t);
            Dc = massMdl.calcContFeedthroughMatrix(x, u, t);
            Rc = massMdl.calcContResidualMatrix();
            yc = Cc * x + Dc * u + Rc;
        end
        function yd = calcDiscOutputDeputyMass(massMdl, x, u, t, Ts)
            % Output of total deputy mass
            Cd = massMdl.calcDiscOutputMatrixDeputyMass(x, u, t, Ts);
            Dd = massMdl.calcDiscFeedthroughMatrix(x, u, t, Ts);
            Rd = massMdl.calcDiscResidualMatrix(x, u, t, Ts);
            yd = Cd * x + Dd * u + Rd;
        end
    end
    methods (Access = protected)
        function Ac = calcContStateMatrix(~, ~, ~, ~)
            % Expelled propellant state matrix
            Ac = 0;
        end
        function Bc = calcContControlMatrix(massMdl, ~, x, u, ~)
            % Expelled propellant control matrix
            % x is the mass of expelled propellant
            mF0 = massMdl.mf0;
            
            if x < mF0
                sImp = massMdl.si;
                eg = massMdl.g; % Gravity of Earth
                Bc = [1/(eg*sImp), 1/(eg*sImp), 1/(eg*sImp)];
                for i = 1:3
                    if u(i) < 0
                        Bc(i) = -Bc(i);
                    end
                end
            else
                Bc = zeros(1,3);
            end
        end
        function Cc = calcContOutputMatrixDeputyMass(~, ~, ~, ~)
            % Output matrix of the total deputy spacecraft mass
            Cc = -1;
        end
        function Cd = calcDiscOutputMatrixDeputyMass(massMdl, x, u, t, ~)
            Cc = massMdl.calcContOutputMatrixDeputyMass(x, u, t);
            Cd = Cc;
        end
        function Ccep = calcContOutputMatrix(~, ~, ~, ~)
            % Output matrix of the expelled propellant
            Ccep = 1;
        end
        function Dc = calcContFeedthroughMatrix(~, ~, ~, ~)
            Dc = [0, 0, 0];
        end
        function Rc = calcContResidualMatrix(massMdl, ~, ~, ~)
            mDry = massMdl.mdry;
            mF0 = massMdl.mf0;
            Rc = mDry + mF0;
        end
        function Rd = calcDiscResidualMatrix(nerm, x, u, t, ~)
            Rc = nerm.calcContResidualMatrix(x, u, t);
            Rd = Rc;
        end
    end
end
