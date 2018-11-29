classdef IndivStateSpaceModel < GeneralMath
    %	Individual class for storing state space models data and methods
    %   Detailed explanation goes here
    
    properties (Constant = true, GetAccess = private)
        PsiSeriesOrder = 5; 
    end
        
    methods (Abstract, Access = protected)
        calcContStateMatrix(model, x, u, t) % Ac
        calcContControlMatrix(model, param, x, u, t) % Bc
        calcContOutputMatrix(model, x, u, t) % Cc
        calcContFeedthroughMatrix(model, x, u, t) % Dc
    end
    
    methods
        function dx = calcDerivatives(model, param, x, u, t)
            Ac = model.calcContStateMatrix(x, u, t);
            Bc = model.calcContControlMatrix(param, x, u, t);
            dx = Ac*x + Bc*u;
        end
        function yc = calcContOutput(model, x, u, t)
            Cc = model.calcContOutputMatrix(x, u, t);
            Dc = model.calcContFeedthroughMatrix(x, u, t);
            yc = Cc*x + Dc*u;
        end
        function xkp1 = calcNextState(model, param, x, u, t, Ts)
            Ad = model.calcDiscStateMatrix(x, u, t, Ts);
            Bd = model.calcDiscControlMatrix(param, x, u, t, Ts);
            xkp1 = Ad * x + Bd * u;
        end
        function yd = calcDiscOutput(model, x, u, t, Ts)
            Cd = model.calcDiscOutputMatrix(x, u, t, Ts);
            Dd = model.calcDiscFeedthroughMatrix(x, u, t, Ts);
            yd = Cd * x + Dd * u;
        end
        function ContSsMdl = calcContStateSpaceModel(model, param, x, u, t)
            Ac = model.calcContStateMatrix(x, u, t);
            Bc = model.calcContControlMatrix(param, x, u, t);
            Cc = model.calcContOutputMatrix(x, u, t);
            Dc = model.calcContFeedthroughMatrix(x, u, t);
            ContSsMdl = ss(Ac, Bc, Cc, Dc);
        end
        function DiscSsMdl = calcDiscStateSpaceModel(model, param, x, u, t, Ts)
            ContSsMdl = model.calcContStateSpaceModel(param, x, u, t);
            DiscSsMdl = c2d(ContSsMdl, Ts);
        end
        function [Ac, Bc, Cc, Dc] = calcContSystemMatrices(model, param, x, u, t)
            Ac = model.calcContStateMatrix(x, u, t);
            Bc = model.calcContControlMatrix(param, x, u, t);
            Cc = model.calcContOutputMatrix(x, u, t);
            Dc = model.calcContFeedthroughMatrix(x, u, t);
        end
        function [Ad, Bd, Cd, Dd] = calcDiscSystemMatrices(model, param, x, u, t, Ts)
            Ad = model.calcDiscStateMatrix(x, u, t, Ts);
            Bd = model.calcDiscControlMatrix(param, x, u, t, Ts);
            Cd = model.calcDiscOutputMatrix(x, u, t, Ts);
            Dd = model.calcDiscFeedthroughMatrix(x, u, t, Ts);
        end
        function [ob] = checkObservability(model, param, x, u, t, Ts)
            DiscSsMdl = model.calcDiscStateSpaceModel(param, x, u, t, Ts);
            obm = obsv(DiscSsMdl);
            ob = rank(obm);
        end
        function [co] = checkControllability(model, param, x, u, t, Ts)
            DiscSsMdl = model.calcDiscStateSpaceModel(param, x, u, t, Ts);
            com = ctrb(DiscSsMdl);
            co = rank(com);
        end
        function [negativeRealParts] = checkStability(model, x, u, t)
            Ac = model.calcContStateMatrix(x, u, t);
            poles = eig(Ac);
            negativeRealParts = 0;
            for i = 1:size(Ac)
                if poles(i) < 0
                    negativeRealParts = negativeRealParts + 1;
                end
            end
        end
        function [Acn, Bcn, Ccn, Dcn] = calcNormalisedContStateSpaceModel(model, Urange, param, x, u, t)
            [Ac, Bc, Cc, Dc] = model.calcContSystemMatrices(param, x, u, t);

            Nu = model.calcControlScalingMatrix(Urange);
            Nx = model.calcStateScalingMatrix();
            Ny = model.calcOutputScalingMatrix();
            
            Acn = Nx \ Ac * Nx;
            Bcn = Nx \ Bc * Nu;
            Ccn = Ny \ Cc * Nx;
            Dcn = Ny \ Dc * Nu;
        end
        function Acn = calcNormalisedContStateMatrix(model, x, u, t)
            Nx = model.calcStateScalingMatrix();
            Ac = model.calcContStateMatrix(x, u, t);
            Acn = Nx \ Ac * Nx;
        end
        function Bcn = calcNormalisedContControlMatrix(model, Urange, param, x, u, t)
            Nu = model.calcControlScalingMatrix(Urange);
            Nx = model.calcStateScalingMatrix();
            Bc = model.calcContControlMatrix(param, x, u, t);
            Bcn = Nx \ Bc * Nu;
        end
        function Ccn = calcNormalisedContOutputMatrix(model, x, u, t)
            Nx = model.calcStateScalingMatrix();
            Ny = model.calcOutputScalingMatrix();
            Cc = model.calcContOutputMatrix(x, u, t);
            Ccn = Ny \ Cc * Nx;
        end
        function Dcn = calcNormalisedContFeedthroughMatrix(model, Urange, x, u, t)
            Nu = model.calcControlScalingMatrix(Urange);
            Ny = model.calcOutputScalingMatrix();
            Dc = model.calcContFeedthroughMatrix(x, u, t);
            Dcn = Ny \ Dc * Nu;
        end
        function Ddn = calcNormalisedDiscFeedthroughMatrix(model, Nu, Ny, x, u, t, Ts)
            Dd = model.calcDiscFeedthroughMatrix(x, u, t, Ts);
            Ddn = Ny \ Dd * Nu;
        end
        function [un] = calcNormalisedControlVector(model, u, Urange)
            Nu = model.calcControlScalingMatrix(Urange);
            un = Nu \ u;
        end
        function [u] = calcUnnormalisedControlVector(model, un, Urange)
            Nu = model.calcControlScalingMatrix(Urange);
            u = Nu * un;
        end
        function [xn] = calcNormalisedStateVector(model, x)
            Nx = model.calcStateScalingMatrix();
            xn = Nx \ x;
        end
        function [x] = calcUnnormalisedStateVector(model, xn)
            Nx = model.calcStateScalingMatrix();
            x = Nx * xn;
        end
        function [yn] = calcNormalisedOutputVector(model, y)
            Ny = model.calcOutputScalingMatrix();
            yn = Ny \ y;
        end
        function [y] = calcUnnormalizedOutputVector(model, yn)
            Ny = model.calcOutputScalingMatrix();
            y = Ny * yn;
        end
    end
    
    methods (Access = protected)
        function Ad = calcDiscStateMatrix(model, x, u, t, Ts)
            Ac = model.calcContStateMatrix(x, u, t);
            Psi = model.calcPsiSeriesSum(Ac, Ts);
            Ad = eye(size(Ac)) + Ac*Ts*Psi;
        end
        function Bd = calcDiscControlMatrix(model, param, x, u, t, Ts)
            Ac = model.calcContStateMatrix(x, u, t);
            Bc = model.calcContControlMatrix(param, x, u, t);
            Psi = model.calcPsiSeriesSum(Ac, Ts);
            Bd = Psi * Ts * Bc;
        end
        function Cd = calcDiscOutputMatrix(model, x, u, t, ~)
            Cc = model.calcContOutputMatrix(x, u, t);
            Cd = Cc;
        end
        function Dd = calcDiscFeedthroughMatrix(model, x, u, t, ~)
            Dc = model.calcContFeedthroughMatrix(x, u, t);
            Dd = Dc;
        end
        function Psi = calcPsiSeriesSum(ssmodel, Ac, Ts)
            Order = ssmodel.PsiSeriesOrder;
            Psi = eye(size(Ac));
            for k = 1:Order
                Psi = Psi + ((Ac*Ts)^k / factorial(k+1));
            end
        end
    end
    methods (Static)
        function [Ac, Bc, Cc, Dc] = findContSeriesConnection(system1, system2, param1, x1, param2, x2, u, t)
            % The output of system1 is the input of system2
            % Overall state vector: x = [dx2; dx1];
            % Overall output vector: y = [y2; y1];
            % Matlab equivalent: sys = series(sys1,sys2,outputs1,inputs2)
            A1 = system1.calcContStateMatrix(x1, u, t);
            B1 = system1.calcContControlMatrix(param1, x1, u, t);
            C1 = system1.calcContOutputMatrix(x1, u, t);
            D1 = system1.calcContFeedthroughMatrix(x1, u, t);
            
            A2 = system2.calcContStateMatrix(x2, u, t);
            B2 = system2.calcContControlMatrix(param2, x2, u, t);
            C2 = system2.calcContOutputMatrix(x2, u, t);
            D2 = system2.calcContFeedthroughMatrix(x2, u, t);
            
            [~, cA2] = size(A2);
            [~, cB2C1] = size(B2*C1);
            [rA1, cA1] = size(A1);
            Azs = zeros(rA1, cA2 + cB2C1 - cA1);
            [~, cC2] = size(C2);
            [~, cD2C1] = size(D2*C1);
            [rC1, cC1] = size(C1);
            Czs = zeros(rC1, cC2 + cD2C1 - cC1);
            
            Ac = [A2 B2*C1; Azs A1];
            Bc = [B2*D1; B1];
            Cc = [C2 D2*C1; Czs C1];
            Dc = [D2*D1; D1];
        end
        function [Ac, Bc, Cc, Dc] = findContSeriesConnection2(system1, MatlSsSystem2, param1, x1, u, t)
            % The output of system1 is the input of system2
            % system1 is i.e. IndivStateSpaceModel object
            % MatlSsSystem2 is Matlab ss object
            % Overall state vector: x = [dx2; dx1];
            % Overall output vector: y = [y2; y1];
            % Matlab equivalent: sys = series(sys1,sys2,outputs1,inputs2)
            A1 = system1.calcContStateMatrix(x1, u, t);
            B1 = system1.calcContControlMatrix(param1, x1, u, t);
            C1 = system1.calcContOutputMatrix(x1, u, t);
            D1 = system1.calcContFeedthroughMatrix(x1, u, t);
            
            A2 = MatlSsSystem2.A;
            B2 = MatlSsSystem2.B;
            C2 = MatlSsSystem2.C;
            D2 = MatlSsSystem2.D;
            
            [~, cA2] = size(A2);
            [~, cB2C1] = size(B2*C1);
            [rA1, cA1] = size(A1);
            Azs = zeros(rA1, cA2 + cB2C1 - cA1);
            [~, cC2] = size(C2);
            [~, cD2C1] = size(D2*C1);
            [rC1, cC1] = size(C1);
            Czs = zeros(rC1, cC2 + cD2C1 - cC1);
            
            Ac = [A2 B2*C1; Azs A1];
            Bc = [B2*D1; B1];
            Cc = [C2 D2*C1; Czs C1];
            Dc = [D2*D1; D1];
        end
        function ContSsMdl = findContYConnection(A1, B1, C1, D1, A2, B2, C2, D2)
            % Both systems have common input
            % system1 is described by A1, B1, C1, D1
            % system2 is described by A2, B2, C2, D2
            % Overall state vector: x = [dx2; dx1];
            % Overall output vector: y = [y2; y1];
            
            [~, cA2] = size(A2);
            [rA1, ~] = size(A1);
            Azs = zeros(rA1, cA2);
            [~, cC2] = size(C2);
            [rC1, ~] = size(C1);
            Czs = zeros(rC1, cC2);
            
            Ac = [A2 Azs'; Azs A1];
            Bc = [B2; B1];
            Cc = [C2 Czs'; Czs C1];
            Dc = [D2; D1];
            
            ContSsMdl = ss(Ac, Bc, Cc, Dc);
        end
    end
end
