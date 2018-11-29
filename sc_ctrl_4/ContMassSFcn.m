function ContMassSFcn(block)
% Continuous model of  the deputy spacecraft mass
% Level-2 MATLAB file S-Function.

  setup(block);  
  
%endfunction

function setup(block)
  
  %% Register number of dialog parameters   
  block.NumDialogPrms = 1;

  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 2;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 3;
  block.InputPort(1).DirectFeedthrough = true;
  
  block.OutputPort(1).Dimensions       = 1;
  block.OutputPort(2).Dimensions       = 1;
  
  %% Set block sample time to continuous
  block.SampleTimes = [0 0];
  
  %% Setup Dwork
  block.NumContStates = 1;

  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Derivatives',             @Derivative);  
  
%endfunction

function SetInpPortFrameData(block, idx, fd)  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  block.OutputPort(2).SamplingMode = fd;
%endfunction

function InitConditions(block)

  %% Initialize Dwork
  SimScenario = block.DialogPrm(1).Data; % Simulation class object
  block.ContStates.Data = SimScenario.MassModel.expf0; % Mass of expelled propellant at t0
%endfunction

function Output(block)
  expelledPropellant = block.ContStates.Data;
  forceVector = block.InputPort(1).Data;
  t = block.CurrentTime;
  SimScenario = block.DialogPrm(1).Data; % Simulation class object
  block.OutputPort(1).Data = SimScenario.MassModel.calcContOutputDeputyMass(expelledPropellant, forceVector, t);
  block.OutputPort(2).Data = SimScenario.MassModel.calcContOutput(expelledPropellant, forceVector, t);
%endfunction

function Derivative(block)
  expelledPropellant = block.ContStates.Data;
  forceVector = block.InputPort(1).Data;
  t = block.CurrentTime;
  SimScenario = block.DialogPrm(1).Data; % Simulation class object
  % Derivative of the expelled fuel mass
  block.Derivatives.Data = SimScenario.MassModel.calcDerivatives(0, expelledPropellant, forceVector, t);
%endfunction

