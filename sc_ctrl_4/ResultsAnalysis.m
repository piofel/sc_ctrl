classdef ResultsAnalysis < handle
    %   Class for visualisation and plotting of the results
    %   Detailed explanation goes here
    
    properties (SetAccess = private, GetAccess = public)
        ControlHistory
        OutputHistory
        SimulationScenario
    end
    
    properties (SetAccess = private, GetAccess = private)
        FigureCounter
    end
    
    properties (Constant = true, GetAccess = private)
        MovieFrameRate = 2;
    end
    
    properties (Constant = true, GetAccess = public)
        VisualisationScaleFactor = 1e-6; % 1000 km
    end
    
    methods
        function ResAnalysis = ResultsAnalysis(simulationScenario) %Constructor
            ResAnalysis.SimulationScenario = simulationScenario;
            ResAnalysis.FigureCounter = 0;
            if exist('control.mat', 'file') == 2
                 tmp = load('control.mat', 'ControlHistory');
                 ResAnalysis.ControlHistory = tmp.ControlHistory;
                 clear tmp
            else
                error('control.mat file does not exist!');
            end
            if exist('output.mat', 'file') == 2
                tmp = load('output.mat', 'OutputHistory');
                ResAnalysis.OutputHistory = tmp.OutputHistory;
                clear tmp
            else
                error('output.mat file does not exist!');
            end
        end
        function [Rc] = calcChiefPositionHistory(ResAnalysis, t)
            % Calculates GEF position of the chief spacecraft
            ChiefGEF = GEF(ResAnalysis.SimulationScenario.RelMotModel.ChiefOrbit);
            [Rc, ~] = ChiefGEF.calcStateVector(t);
        end
        function [Rd] = calcDeputyPositionHistory(ResAnalysis, StateVectorType, t)
            % Calculates GEF position of the deputy spacecraft
            if strcmp(StateVectorType, 'LVLH')
                StateVector = ResAnalysis.findStateVectorAtMomentT(t);
                LvlhStateVector = StateVector(1:6);
                ChiefGEF = GEF(ResAnalysis.SimulationScenario.RelMotModel.ChiefOrbit);
                [Rd, ~] = ChiefGEF.calcDeputyStateVectorFromLVLH(LvlhStateVector, t);
            elseif strcmp(StateVectorType, 'ROE')
                StateVector = ResAnalysis.findStateVectorAtMomentT(t);
                ROE = StateVector(1:6); % Relative orbital elements
                CurrentDeputyOrbit = ResAnalysis.SimulationScenario.RelMotModel.calcCurrentDeputyOrbit(ROE', t);
                DeputyGEF = GEF(CurrentDeputyOrbit);
                [Rd, ~] = DeputyGEF.calcStateVector(t);
            end
        end
        function [Rc0] = calcChiefInitPosition(ResAnalysis)
            % Calculates GEF position of the chief spacecraft at t0
            t0 = ResAnalysis.SimulationScenario.StartTime;
            ChiefGEF = GEF(ResAnalysis.SimulationScenario.RelMotModel.ChiefOrbit);
            [Rc0, ~] = ChiefGEF.calcStateVector(t0);
        end
        function [Rd0] = calcDeputyInitPosition(ResAnalysis, StateVectorType)
            % Calculates GEF position of the deputy spacecraft at t0
            t0 = ResAnalysis.SimulationScenario.StartTime;
            if strcmp(StateVectorType, 'LVLH')
                StateVector = ResAnalysis.findStateVectorAtMomentT(t0);
                LvlhStateVector = StateVector(1:6);
                ChiefGEF = GEF(ResAnalysis.SimulationScenario.RelMotModel.ChiefOrbit);
                [Rd0, ~] = ChiefGEF.calcDeputyStateVectorFromLVLH(LvlhStateVector, t0);
            elseif strcmp(StateVectorType, 'ROE')
                StateVector = ResAnalysis.findStateVectorAtMomentT(t0);
                ROE = StateVector(1:6); % Relative orbital elements
                CurrentDeputyOrbit = ResAnalysis.SimulationScenario.RelMotModel.calcCurrentDeputyOrbit(ROE', t0);
                DeputyGEF = GEF(CurrentDeputyOrbit);
                [Rd0, ~] = DeputyGEF.calcStateVector(t0);
            end
        end
        function plotControlHistory(ResAnalysis)            
            ResAnalysis.FigureCounter = ResAnalysis.FigureCounter + 1;
            figure(ResAnalysis.FigureCounter);
            
            t  = ResAnalysis.ControlHistory.Time;
            u1 = ResAnalysis.ControlHistory.Data(:,1);
            u2 = ResAnalysis.ControlHistory.Data(:,2);
            u3 = ResAnalysis.ControlHistory.Data(:,3);
            
            ResAnalysis.createSubplot(3, 1, 1, t, u1, 'u1, N');
            title('Control history');
            ResAnalysis.createSubplot(3, 1, 2, t, u2, 'u2, N');
            ResAnalysis.createSubplot(3, 1, 3, t, u3, 'u3, N');
            saveas(gcf, 'plots\control', 'png');
        end
        function plotLvlhTrajectory(ResAnalysis)            
            ResAnalysis.FigureCounter = ResAnalysis.FigureCounter + 1;
            figure(ResAnalysis.FigureCounter);
            
            t  = ResAnalysis.OutputHistory.Time;
            x1 = ResAnalysis.OutputHistory.Data(:,1);
            x2 = ResAnalysis.OutputHistory.Data(:,2);
            x3 = ResAnalysis.OutputHistory.Data(:,3);
            
            ResAnalysis.createSubplot(3, 1, 1, t, x1, 'x1, m');
            title('Relative position in LVLH frame');
            ResAnalysis.createSubplot(3, 1, 2, t, x2, 'x2, m');
            ResAnalysis.createSubplot(3, 1, 3, t, x3, 'x3, m');
            saveas(gcf, 'plots\position', 'png');
            
            ResAnalysis.FigureCounter = ResAnalysis.FigureCounter + 1;
            figure(ResAnalysis.FigureCounter);
            
            x4 = ResAnalysis.OutputHistory.Data(:,4);
            x5 = ResAnalysis.OutputHistory.Data(:,5);
            x6 = ResAnalysis.OutputHistory.Data(:,6);
            
            ResAnalysis.createSubplot(3, 1, 1, t, x4, 'x4, m/s');
            title('Relative velocity in LVLH frame');
            ResAnalysis.createSubplot(3, 1, 2, t, x5, 'x5, m/s');
            ResAnalysis.createSubplot(3, 1, 3, t, x6, 'x6, m/s');
            saveas(gcf, 'plots\velocity', 'png');
        end
        function plotRelativeOrbitalElementsTrajectory(ResAnalysis)
            ResAnalysis.FigureCounter = ResAnalysis.FigureCounter + 1;
            figure(ResAnalysis.FigureCounter);
            
            t  = ResAnalysis.OutputHistory.Time;
            x1 = ResAnalysis.OutputHistory.Data(:,1);
            x2 = ResAnalysis.OutputHistory.Data(:,2);
            x3 = ResAnalysis.OutputHistory.Data(:,3);
            
            ResAnalysis.createSubplot(3, 1, 1, t, x1, 'Semimajor axis, m');
            title('Relative orbital elements 1 - 3');
            ResAnalysis.createSubplot(3, 1, 2, t, x2, 'Eccentricity');
            ResAnalysis.createSubplot(3, 1, 3, t, x3, 'Inclination, rad');
            saveas(gcf, 'plots\rel_oe_13', 'png');
            
            ResAnalysis.FigureCounter = ResAnalysis.FigureCounter + 1;
            figure(ResAnalysis.FigureCounter);
            
            x4 = ResAnalysis.OutputHistory.Data(:,4);
            x5 = ResAnalysis.OutputHistory.Data(:,5);
            x6 = ResAnalysis.OutputHistory.Data(:,6);
            
            ResAnalysis.createSubplot(3, 1, 1, t, x4, 'RAAN, rad');
            title('Relative orbital elements 4 - 6');
            ResAnalysis.createSubplot(3, 1, 2, t, x5, 'Arg. of periapsis, rad');
            ResAnalysis.createSubplot(3, 1, 3, t, x6, 'Mean anomaly, rad');
            saveas(gcf, 'plots\rel_oe_46', 'png');
        end        
        function plotExpelledPropellantMass(ResAnalysis)            
            ResAnalysis.FigureCounter = ResAnalysis.FigureCounter + 1;
            figure(ResAnalysis.FigureCounter);
            
            t  = ResAnalysis.OutputHistory.Time;
            x7 = ResAnalysis.OutputHistory.Data(:,7);
            
            ResAnalysis.createSubplot(1, 1, 1, t, x7, 'x7, kg');
            title('Mass of the expelled propellant');
            saveas(gcf, 'plots\mass', 'png');
        end
        function createMovie(ResAnalysis)
            if exist('video_data.mat', 'file') == 2
                load('video_data.mat');
                writerObj = VideoWriter('maneuver_visualization.avi');
                writerObj.FrameRate = ResAnalysis.MovieFrameRate;
                open(writerObj);
                writeVideo(writerObj, video_data.Data);
                close(writerObj);
            else
                error('video_data.mat file does not exist!');
            end
        end
    end
    
    methods (Access = private)
        function StateVector = findStateVectorAtMomentT(ResAnalysis, t)
            % Search for state vector in OutputHistory using time
            % criterion
            TsInc = ResAnalysis.OutputHistory.TimeInfo.Increment;
            i = 0;
            while TsInc * i < t
                i = i + 1;
            end
            tmp = getsampleusingtime(ResAnalysis.OutputHistory, TsInc * i);
            StateVector = tmp.Data;
            clear tmp
        end
        function [tmin, tmax, ymin, ymax] = calcAxisScale(ResAnalysis, t, y)
            tmin = ResAnalysis.SimulationScenario.StartTime;
            tmax = max(t);
            miny = min(y);
            maxy = max(y);
            if miny||maxy
                ymin = miny - 0.1 * abs(miny);
                ymax = maxy + 0.1 * abs(maxy);
            else
                ymin = -10;
                ymax =  10;
            end
        end
        function createSubplot(ResAnalysis, m, n, p, t, y, yAxisLabel)
            subplot(m, n, p);
            plot(t, y);
            [tmin, tmax, ymin, ymax] = ResAnalysis.calcAxisScale(t, y);
            axis([tmin, tmax, ymin, ymax]);
            grid on
            xlabel('Time, s');
            ylabel(yAxisLabel);
        end
    end
end
