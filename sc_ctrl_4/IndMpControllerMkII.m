classdef IndMpControllerMkII < IndivMPController
    %   Class of Mark II controllers
    %   Detailed explanation goes here
    
    methods (Access = public)
        function DiscModel = calcDiscModel(Controller, x, u, t)
            % Calculates unnormalised discrete model
            % The model contains relative motion model and mass model
            Ts = Controller.Ts;            
            x_rm = x(1:6);
            x_ep = x(7);
            
            md = Controller.MassModel.calcContOutputDeputyMass(x_ep, u, t);            
            [Ac_rm, Bc_rm, Cc_rm, Dc_rm] = ... % Linear term of the relative motion model
            Controller.RelMotModel.calcContSystemMatrices(md, x_rm, u, t);      
            
            [Ac_ep, Bc_ep, Cc_ep, Dc_ep] = ...  % Mass model
            Controller.MassModel.calcContSystemMatrices([], x_ep, u, t);
            
            ContConnectedMatlSsMdl = ...
            IndivStateSpaceModel.findContYConnection(Ac_ep, Bc_ep, Cc_ep, Dc_ep, ...
                                                     Ac_rm, Bc_rm, Cc_rm, Dc_rm);

            DiscModel = c2d(ContConnectedMatlSsMdl, Ts);
        end
    end
    methods (Access = protected)
        function BarR = calcControlWeightMatrix(mpController, m)
            Nc = mpController.Parameters.ControlHorizon;            
            cw1 = mpController.Parameters.ControlWeight(1);
            cw2 = mpController.Parameters.ControlWeight(2);
            cw3 = mpController.Parameters.ControlWeight(3);
            
            BarR = zeros(m*Nc, m*Nc);
            BarR(1:m, 1:m) = diag([cw1 cw2 cw3]);
            for i = 2:Nc
                BarR((i-1)*m+1:i*m, (i-1)*m+1:i*m) = diag([cw1 cw2 cw3]);
            end
        end
        function [M1, N1] = calcControlAmplitudeConstraints(mpController, m, ukm1)
            Nc = mpController.Parameters.ControlHorizon;
            
            % Physical control amplitude constraints on u1, u2 and u3
            physical_ctrl_amp_cst =  mpController.Parameters.ControlConstraint;
            physical_ctrl_amp_cst_vec = physical_ctrl_amp_cst * ones(3,1);
            
            u_max = physical_ctrl_amp_cst_vec; 
            u_min = -u_max;
            
            M1dw = zeros(m*Nc, m*Nc);
            M1dw(1:m, 1:m) = eye(m);
            for i = 2:Nc
                M1dw((i-1)*m+1:i*m, 1:m) = eye(m);
            end
            for j = 2:Nc
                for k = 2:Nc
                    if j >= k
                        M1dw((j-1)*m+1:j*m, (k-1)*m+1:k*m) = eye(m);
                    end
                end
            end
            M1up = -M1dw;
            M1 = [M1up; M1dw];
            
            C1 = eye(m);
            for i = 2:Nc
                C1((i-1)*m+1:i*m, 1:m) = eye(m);
            end
            
            N1 = [-C1*u_min + C1*ukm1;
                   C1*u_max - C1*ukm1];
        end
    end
end
