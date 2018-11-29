function ContRelMotSFcn(block)
% Continuous relative motion model
% Level-2 MATLAB file S-Function.

  setup(block);  
  
%endfunction

function setup(block)
  
  %% Register number of dialog parameters   
  block.NumDialogPrms = 1;

  %% Register number of input and output ports
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 3;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = 1;
  block.InputPort(2).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 6;
  
  %% Set block sample time to continuous
  block.SampleTimes = [0 0];
  
  %% Setup Dwork
  block.NumContStates = 6;

  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Derivatives',             @Derivative);  
  
%endfunction

function InitConditions(block)

  %% Initialize Dwork
  SimScenario = block.DialogPrm(1).Data; % Simulation class object
  block.ContStates.Data = SimScenario.RelMotModel.x0;  
  
%endfunction

function Output(block)
  SimScenario = block.DialogPrm(1).Data; % Simulation class object
  x = block.ContStates.Data;
  u = block.InputPort(1).Data;
  t = block.CurrentTime;
  block.OutputPort(1).Data = SimScenario.RelMotModel.calcContOutput(x, u, t);
  
%endfunction

function Derivative(block)
  SimScenario = block.DialogPrm(1).Data; % Simulation class object
  md = block.InputPort(2).Data; % Current mass of spacecraft
  mdry = SimScenario.MassModel.mdry; % Mass of spacecraft without fuel
  x = block.ContStates.Data;
  t = block.CurrentTime;
  
  if md > mdry
    u = block.InputPort(1).Data;
  else
    u = [0; 0; 0];
  end
  
  block.Derivatives.Data = SimScenario.RelMotModel.calcDerivatives(md, x, u, t);
  
%endfunction

