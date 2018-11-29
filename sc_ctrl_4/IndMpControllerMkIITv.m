classdef IndMpControllerMkIITv < IndMpControllerMkII
    %   Class of Mark II controllers using prediction with time varying model
    %   Detailed explanation goes here
    
    properties
        hce_val % Heuristic control estimation value
        hce_algorithm % Heuristic control estimation algorithm
    end
    
    methods (Access = public)
        function mpController = ... % Constructor
            IndMpControllerMkIITv(mpcParameters, reference, relMotMdl, massMdl, ~, ~, hceAlgorithm, ts)
            mpController.Ts = ts;
            mpController.Parameters = mpcParameters;
            mpController.Reference = reference;
            mpController.RelMotModel = relMotMdl;
            mpController.MassModel = massMdl;

            mpController.PauseTime = 3600000;
            
            mpController.hce_algorithm = hceAlgorithm;
            mpController.hce_val = [0; 0; 0];
        end
        function u = calcControllerMove(mpController, x_hat, X_hat, ukm1, t)
            % Test
            % ---------------------------------------------------
            if t >= mpController.PauseTime
                keyboard
            end
            % ---------------------------------------------------
            [As, Bs, Cs, m, n, q] = mpController.calcFutureTvModelMatrices(x_hat, mpController.hce_val, t);
            [PhiT_Phi, PhiT_F, PhiT_BarRs] = mpController.calcMpcGainUsingTvm(As, Bs, Cs, m, n, q);

            BarR = mpController.calcControlWeightMatrix(m);
            r = mpController.Reference';
            
            H = PhiT_Phi + BarR; % Hessian
            f = -(PhiT_BarRs * r - PhiT_F * X_hat);

            [M1, N1] = mpController.calcControlAmplitudeConstraints(m, ukm1);      

            % [DeltaU] = mpController.QpHildreth(H, f, M1, N1);
            [DeltaU, ~] = quadprog(H, f, M1, N1);
            deltau = mpController.calcFirstControlIncrement(m, DeltaU);
            u = ukm1 + deltau;
            
            mpController.hce_val = mpController.hce_algorithm.calcOutput(u);
            
            % Test
            % ---------------------------------------------------
            kappa = cond(H);
            display(kappa, 'Hessian condition number');
            % ---------------------------------------------------
        end
    end
    
    methods (Access = private)
        function [As, Bs, Cs, m, n, q] = calcFutureTvModelMatrices(mpController, x_hat, hce_val, t)
            % Function calculates set of future model matrices
            Ts = mpController.Ts;
            Np = mpController.Parameters.PredictionHorizon;
                        
            % Calculate A(k), B(k), C(k) (for i = 0)
            DiscMatlSsMdl = mpController.calcDiscModel(x_hat, hce_val, t);
            [Aa, Ba, Ca] = mpController.calcAugmentedModel(DiscMatlSsMdl);
            
            [~, m] = size(Ba); % Number of inputs
            [n, ~] = size(Aa); % Number of states
            [q, ~] = size(Ca); % Number of outputs
            
            As(1:n, :) = Aa;
            Bs(1:n, :) = Ba;
            Cs(1:q, :) = Ca;
            
            x = x_hat;
            u = hce_val;

            % Calculate A(k+i), B(k+i), C(k+i) (for i >= 1)
            for i=1:Np-1
               % Next state is calculated basing on current state
               x_rm = x(1:6);
               x_ep = x(7);
               md = mpController.MassModel.calcContOutputDeputyMass(x_ep, u, (t + (i-1)*Ts));
               x_rm = mpController.RelMotModel.calcNextState(md, x_rm, u, (t + (i-1)*Ts), Ts);
               x_ep = mpController.MassModel.calcNextState([], x_ep, u, (t + (i-1)*Ts), Ts);
               x = [x_rm; x_ep]; % x(k+i)
               % u = ukm1 * exp(-(i-1)*alpha/Np);
               
               % Next matrices set is formulated
               DiscMatlSsMdl = mpController.calcDiscModel(x, u, (t + i*Ts));
               [Aa, Ba, Ca] = mpController.calcAugmentedModel(DiscMatlSsMdl);
               As(i*n+1:i*n+n, :) = Aa;
               Bs(i*n+1:i*n+n, :) = Ba;
               Cs(i*q+1:i*q+q, :) = Ca;
            end
        end
        function [PhiT_Phi, PhiT_F, PhiT_BarRs] = calcMpcGainUsingTvm(mpController, As, Bs, Cs, m, n, q)
            % Modified calcMpcGain function enabling use of time varying model
            % m - number of inputs
            % n - number of states
            % q - number of outputs
            % As, Bs, Cs - sets of augmented discrete state-space model matrices

            Nc = mpController.Parameters.ControlHorizon;
            Np = mpController.Parameters.PredictionHorizon;
            
            % Calculate F
            AsQuotient = As(1:n, :);
            F(1:q, :) = Cs(1:q, :) * AsQuotient; % C(k) * A(k)
            for i = 1:Np-1
                AsQuotient = As(i*n+1:i*n+n, :) * AsQuotient;
                F(i*q+1:i*q+q, :) = Cs(i*q+1:i*q+q, :) * AsQuotient;
            end
            
            % Calculate Phi
            Phi = zeros(q*Np, m*Nc); % Declare the dimension of Phi
            Phi(1:q, 1:m) = Cs(1:q, :) * Bs(1:n, :); % C(k) * B(k)
            AsQuotient = eye(n);
            for i = 1:Np-1
                AsQuotient = As(i*n+1:i*n+n, :) * AsQuotient;
                Phi(i*q+1:i*q+q, 1:m) = Cs(i*q+1:i*q+q, :) * AsQuotient * Bs(1:n, :);
            end
            for j = 1:Nc-1
                Phi(1:q, j*m+1:j*m+m) = zeros(q, m);
            end
            for j = 1:Nc-1
                AsQuotient = eye(n);
                for i = 1:Np-1
                    if i < j
                        Phi(i*q+1:i*q+q, j*m+1:j*m+m) = zeros(q, m);
                    elseif i == j
                        Phi(i*q+1:i*q+q, j*m+1:j*m+m) = ... 
                            Cs(i*q+1:i*q+q, :) * Bs(j*n+1:j*n+n, :);
                    else
                        AsQuotient = As(i*n+1:i*n+n, :) * AsQuotient;
                        Phi(i*q+1:i*q+q, j*m+1:j*m+m) = ... 
                            Cs(i*q+1:i*q+q, :) * AsQuotient * Bs(j*n+1:j*n+n, :);
                    end
                end
            end
            
            BarRs(1:q, :) = eye(q);
            for kkk = 2:Np
                BarRs((kkk-1)*q+1:kkk*q, :) = eye(q);
            end

            PhiT_Phi = Phi' * Phi;
            PhiT_F = Phi' * F;
            PhiT_BarRs = Phi' * BarRs;
        end
    end
end
