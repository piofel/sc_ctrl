clear all
close all
clc

%% Sampling parameters
Ts = 300;
ControlRecorderTs = '300';
TrajectoryRecorderTs = '300';

%% Visualisation parameters
VideoRecorderTs = '1200';

%% Simulation parameters
InitialTimeMoment = 0;
SimulationTime = 80000;

%% Relative motion initial conditions
ChiefCoeVector =               [50000 0.8 260.0 60 120.0 80.0];
ChiefCoeUnits =          0; % km & deg (0) or m & rad (1)
DeputyInitialCoeVector =       [35000 0.5 270.0 30 160.0 15.0];
DeputyInitialCoeUnits =  0; % km & deg (0) or m & rad (1)

%% Satellite parameters
DryScMass = 100;
InitFuelMass = 900;
SpecImp = 450;
ExpFuel0 = 0;

%% Controller parameters
Reference = [0 0 0 0 0 0 0];
PredictionHorizon = 70;
ControlHorizon = 5;
ControlConstraint = 100;
ControlWeight = 15*10e10 * [1 1 1];
HceFilterOrder = 10;

%% Disturbance parameters
InputDistAmp = 0; % m/s^2
OutputDistAmp = 0; % m, m/s
DistSampleTime = 1200;
StepTime = 1200;
StepDist = 0;

%% ----------------------------------------------------------------------------------------------------------------
%% ----------------------------------------------------------------------------------------------------------------
% Objects creation
ChiefCoe = COE(ChiefCoeVector, InitialTimeMoment, ChiefCoeUnits);
DeputyInitialCoe = COE(DeputyInitialCoeVector, InitialTimeMoment, DeputyInitialCoeUnits);
ChiefOrbit = KeplerOrbit(ChiefCoe);
DeputyInitialOrbit = KeplerOrbit(DeputyInitialCoe);

RelMotMdl = NermModel(ChiefOrbit, DeputyInitialOrbit, 'DeputyInitOrbit', InitialTimeMoment);
MassMdl = MassModel(DryScMass, InitFuelMass, SpecImp, ExpFuel0);
HceAlgorithm = MovingAverageFilter(HceFilterOrder, 3);
DistModel = DisturbanceModel(InputDistAmp, OutputDistAmp, DistSampleTime);

ControllerParameters = MPControllerParameters(PredictionHorizon, ControlHorizon, ControlConstraint, ...
               0, 0, ControlWeight, 0, 0);            
Controller = IndMpControllerMkIITv(ControllerParameters, Reference, RelMotMdl, MassMdl, ...
    0, 0, HceAlgorithm, Ts);

%% Simulation
SimulationScenario = Simulation(InitialTimeMoment, RelMotMdl, MassMdl, DistModel, Controller, Ts);
if  isempty(find_system('Name','control_system')) % Open the control system
    open_system('control_system');
end
set_param('control_system/control_history_recorder','SampleTime', ControlRecorderTs);
set_param('control_system/trajectory_recorder','SampleTime', TrajectoryRecorderTs);
sim('control_system', SimulationTime); % Run simulation with specified time (2nd param.)

%% Visualisation
ResAnalysis = ResultsAnalysis(SimulationScenario);
if  isempty(find_system('Name','visualisation_system')) % Open the visualisation system
    open_system('visualisation_system');
end
set_param('visualisation_system/video_data_recorder','SampleTime', VideoRecorderTs);
sim('visualisation_system', SimulationTime); % Run simulation with specified time (2nd param.)

ResAnalysis.plotControlHistory();
ResAnalysis.plotLvlhTrajectory();
ResAnalysis.plotExpelledPropellantMass();
ResAnalysis.createMovie();
