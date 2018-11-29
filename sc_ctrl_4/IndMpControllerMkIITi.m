classdef IndMpControllerMkIITi < IndMpControllerMkII
    %   Class of Mark II controllers using prediction with time invariant model
    %   Detailed explanation goes here
    
    methods (Access = public)
        function mpController = ... % Constructor
            IndMpControllerMkIITi(mpcParameters, reference, relMotMdl, massMdl, ~, ~, ~, ts)
            mpController.Ts = ts;
            mpController.Parameters = mpcParameters;
            mpController.Reference = reference;
            mpController.RelMotModel = relMotMdl;
            mpController.MassModel = massMdl;
            
            % Variable used for tests:
            mpController.PauseTime = 10000000;
        end
        function u = calcControllerMove(mpController, x_hat, X_hat, ukm1, t)
            % Test
            % ---------------------------------------------------
            if t >= mpController.PauseTime
                keyboard
            end
            % ---------------------------------------------------
            
            DiscMatlSsMdl = mpController.calcDiscModel(x_hat, ukm1, t);
            [Ade, Bde, Cde] = mpController.calcAugmentedModel(DiscMatlSsMdl);

            [PhiT_Phi, PhiT_F, PhiT_BarRs] = mpController.calcMpcGain(Ade, Bde, Cde);
            
            [~, m] = size(Bde); % Number of inputs
            BarR = mpController.calcControlWeightMatrix(m);
            r = mpController.Reference';

            H = PhiT_Phi + BarR; % Hessian
            f = -(PhiT_BarRs*r - PhiT_F*X_hat);

            [M1, N1] = mpController.calcControlAmplitudeConstraints(m, ukm1);
            
            [DeltaU] = quadprog(H, f, M1, N1);
            deltau = mpController.calcFirstControlIncrement(m, DeltaU);
            u = ukm1 + deltau;
            
            % Test
            % ---------------------------------------------------
            kappa = cond(H);
            display(kappa, 'Hessian condition number');
            % ---------------------------------------------------
        end
    end
end
