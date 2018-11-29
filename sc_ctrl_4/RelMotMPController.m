classdef RelMotMPController < MPController
    %	Class of spacecraft relative motion controllers
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
        RelMotModel
        MassModel
    end
    
    properties (Constant = true, GetAccess = public)
        u0 = [0; 0; 0]; % Initial controller output
    end
    
    methods (Access = public)
        function x_hat_0 = calcInitialStateEstimate(Controller) 
            x_hat_rm_0 = Controller.RelMotModel.x0;
            x_hat_ep_0 = Controller.MassModel.expf0;
            x_hat_0 = [x_hat_rm_0; x_hat_ep_0];
        end
        function x_hat_kp1 = calcStateEstimate(Controller, ym, x_hat, u, t)
            Ts = Controller.Ts;
            
            x_hat_rm = x_hat(1:6);
            x_hat_ep = x_hat(7);
            
            m_hat_d = Controller.MassModel.calcContOutputDeputyMass(x_hat_ep, u, t);
            modelBasedEstimate_rm = Controller.RelMotModel.calcNextState(m_hat_d, x_hat_rm, u, t, Ts);
            modelBasedEstimate_ep = Controller.MassModel.calcNextState([], x_hat_ep, u, t, Ts);
            modelBasedEstimate = [modelBasedEstimate_rm; modelBasedEstimate_ep];
            
            [Ad_rm, ~, Cd_rm, ~] = Controller.RelMotModel.calcDiscSystemMatrices(m_hat_d, x_hat_rm, u, t, Ts);
            [Ad_ep, ~, Cd_ep, ~] = Controller.MassModel.calcDiscSystemMatrices([], x_hat_ep, u, t, Ts);
            A = [Ad_rm, zeros(6,1); zeros(1,6), Ad_ep];
            C = [Cd_rm, zeros(6,1); zeros(1,6), Cd_ep];
            
            Kob = Controller.calcObserverGain(A, C);            
            correctionTerm = Kob * (ym - C*x_hat);
            x_hat_kp1 = modelBasedEstimate + correctionTerm;
        end
    end
    
    methods (Access = protected)
        function Kob = calcObserverGain(~, A, C)
            Kob = 0.7 * eye(7);
        end
    end
end
