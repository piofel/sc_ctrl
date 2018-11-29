classdef IndivMPController < RelMotMPController
    %	Class of individual spacecraft relative motion controllers
    %   Detailed explanation goes here
    
    properties (SetAccess = public, GetAccess = public)
        % Variable used for tests:
        PauseTime
    end
    
    properties (Constant = true, GetAccess = public)
        QpHildrethMaxIterations = 1e4;
        QpHildrethTolerance = 1e-10;
        
        % Variable used for tests:
        InitControlWeight = [100, 100, 100];
    end
    
    methods (Access = public)
        function X_hat_0 = calcInitialAugmentedStateEstimate(mpController)
            Ts = mpController.Ts;            
            x_hat_rm_0 = mpController.RelMotModel.x0;
            x_hat_ep_0 = mpController.MassModel.expf0;
            u0 = mpController.u0;
            t0 = mpController.RelMotModel.t0;
            
            m_hat_d_0 = mpController.MassModel.calcContOutputDeputyMass(x_hat_ep_0, u0, t0);
            [~, ~, Cd_rm, ~] = mpController.RelMotModel.calcDiscSystemMatrices(m_hat_d_0, x_hat_rm_0, u0, t0, Ts);
            [~, ~, Cd_ep, ~] = mpController.MassModel.calcDiscSystemMatrices([], x_hat_ep_0, u0, t0, Ts);
            
            y_hat_rm_0 = Cd_rm * x_hat_rm_0;
            y_hat_ep_0 = Cd_ep * x_hat_ep_0;
            
            y_hat_0 = [y_hat_rm_0; y_hat_ep_0];
            
            X_hat_0 = [zeros(7,1); y_hat_0];
        end
        function X_hat_kp1 = calcAugmentedStateEstimate(Controller, x_hat_kp1, x_hat, u, t)
            Ts = Controller.Ts;

            x_hat_rm_kp1 = x_hat_kp1(1:6);
            x_hat_ep_kp1 = x_hat_kp1(7);
            
            m_hat_d = Controller.MassModel.calcContOutputDeputyMass(x_hat_ep_kp1, u, t+Ts);
            [~, ~, Cd_rm, ~] = Controller.RelMotModel.calcDiscSystemMatrices(m_hat_d, x_hat_rm_kp1, u, t+Ts, Ts);
            [~, ~, Cd_ep, ~] = Controller.MassModel.calcDiscSystemMatrices([], x_hat_ep_kp1, u, t+Ts, Ts);
            
            y_hat_rm = Cd_rm * x_hat_rm_kp1;
            y_hat_ep = Cd_ep * x_hat_ep_kp1;
            y_hat = [y_hat_rm; y_hat_ep];
            
            X_hat_kp1 = [x_hat_kp1 - x_hat; y_hat];
        end
    end
    
    methods (Access = protected)
        function [Ade, Bde, Cde] = calcAugmentedModel(~, DiscMatlSsMdl)
            % Algorithm described in Wang p. 6
            Ad = DiscMatlSsMdl.A;
            Bd = DiscMatlSsMdl.B;
            Cd = DiscMatlSsMdl.C;
            
            [m1, ~] = size(Cd);
            [n1, n_in] = size(Bd);
            Ade = eye(n1+m1, n1+m1);
            Ade(1:n1, 1:n1) = Ad;
            Ade(n1+1:n1+m1, 1:n1) = Cd*Ad;
            Bde = zeros(n1+m1, n_in);
            Bde(1:n1, :) = Bd;
            Bde(n1+1:n1+m1, :) = Cd*Bd;
            Cde=zeros(m1, n1+m1);
            Cde(:, n1+1:n1+m1) = eye(m1, m1);
        end
        function [PhiT_Phi, PhiT_F, PhiT_BarRs] = calcMpcGain(mpController, Ade, Bde, Cde)
            % Algorithm described in Wang p. 13
            % Ade, Bde, Cde are augmented discrete state-space model
            Nc = mpController.Parameters.ControlHorizon;
            Np = mpController.Parameters.PredictionHorizon;
            
            [q, ~] = size(Cde); % Number of outputs
            [~, m] = size(Bde); % Number of inputs

            h(1:q, :) = Cde;
            F(1:q, :) = Cde*Ade;
            for kk = 2:Np
                h((kk-1)*q+1:kk*q, :) = h((kk-2)*q+1:(kk-1)*q, :) * Ade;
                F((kk-1)*q+1:kk*q, :) = F((kk-2)*q+1:(kk-1)*q, :) * Ade;
            end
            
            v = h*Bde;
            Phi = zeros(q*Np, m*Nc); % Declare the dimension of Phi
            Phi(:, 1:m) = v;
            for i = 2:Nc
                Phi(:, (i-1)*m+1:i*m) = [zeros((i-1)*q, m); v(1:(Np-i+1)*q, 1:m)]; % Toeplitz matrix
            end

            BarRs(1:q, :) = eye(q);
            for kkk = 2:Np
                BarRs((kkk-1)*q+1:kkk*q, :) = eye(q);
            end

            PhiT_Phi = Phi' * Phi;
            PhiT_F = Phi' * F;
            PhiT_BarRs = Phi' * BarRs;
        end
        function deltau = calcFirstControlIncrement(mpController, m, DeltaU)
            Nc = mpController.Parameters.ControlHorizon;
            
            deltau = zeros(m, m*Nc);
            deltau(1:m, 1:m) = eye(m);
            for i = 2:Nc
                deltau(:, (i-1)*m+1:i*m) = zeros(m);
            end
            deltau = deltau * DeltaU;
        end
        function lambda = calcLagrangeMultipliers(~, H, f, M, gamma)
            lambda = -(M * (H \ M')) \ (gamma + M * (H \ f));
        end
        function eta = QpHildreth(mpController, H, f, A_cons, b)
            % E=H;
            % F=f;
            % M=A_cons;
            % gamma=b;
            % eta =x
            
            P = A_cons*(H\A_cons');
            d = (A_cons*(H\f) + b);
            [n, m] = size(d);
            x_ini = zeros(n, m);
            lambda = x_ini;
            for km=1 : mpController.QpHildrethMaxIterations
                % Find the elements in the solution vector one by one
                % km could be larger if the Lagranger multiplier has a slow
                % convergence rate.
                lambda_p = lambda;
                for i = 1:n
                    w = P(i,:) * lambda-P(i,i) * lambda(i,1);
                    w = w + d(i,1);
                    la = -w/P(i,i);
                    lambda(i,1) = max(0,la);
                end
                al = (lambda-lambda_p)' * (lambda-lambda_p);
                if (al < mpController.QpHildrethTolerance);
                    break;
                end
                if km == mpController.QpHildrethMaxIterations
                    warning('Hildreth QP algorithm tolerance not satisfied.');
                end
            end
            eta = -H\f - H\A_cons'*lambda;
        end
        function DeltaU_Glob = calcGlobalOptimalSolution(~, PhiT_Phi, BarR, PhiT_BarRs, PhiT_F, X_hat, r)
            DeltaU_Glob = (PhiT_Phi + BarR) \ (PhiT_BarRs*r - PhiT_F*X_hat);
        end
        function calcControlWeights(mpController, x_hat_rm, t)
            alpha1 = 0.0008;
            alpha2 = 100;
            cw_0 = mpController.InitControlWeight;
            cw_vec = cw_0 * (exp(-alpha1*t) + exp(-alpha2*norm(x_hat_rm)));
            mpController.Parameters.ControlWeight = cw_vec;
        end
    end    
end
