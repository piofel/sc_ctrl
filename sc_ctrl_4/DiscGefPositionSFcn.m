function DiscGefPositionSFcn(block)
% Level-2 MATLAB file S-Function.

  setup(block);
  
%endfunction

function setup(block)

  %% Register number of dialog parameters   
  block.NumDialogPrms = 1;
  
  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 4;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
  
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(1).SamplingMode = 'Sample';
  
  block.OutputPort(1).Dimensions = 3;
  block.OutputPort(2).Dimensions = 3;
  block.OutputPort(3).Dimensions = 3;
  block.OutputPort(4).Dimensions = 3;
  block.OutputPort(1).SamplingMode = 'Sample';
  block.OutputPort(2).SamplingMode = 'Sample';
  block.OutputPort(3).SamplingMode = 'Sample';
  block.OutputPort(4).SamplingMode = 'Sample';
  
  %% Set block sample time to inherited
  block.SampleTimes = [-1 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';
  
  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions); 
  block.RegBlockMethod('Outputs',                 @Output);  
  
%endfunction

function DoPostPropSetup(block)

  %% Setup Dwork
  block.NumDworks = 2;
  
  block.Dwork(1).Name = 'Rc0'; % Chief init position
  block.Dwork(1).Dimensions      = 3;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name = 'Rd0'; % Deputy init position
  block.Dwork(2).Dimensions      = 3;
  block.Dwork(2).DatatypeID      = 0;
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
  ResAnalysis = block.DialogPrm(1).Data; % ResultsAnalysis class object
  StateVectorType = ResAnalysis.SimulationScenario.RelMotModel.StateVectorType;
  block.Dwork(1).Data = ResAnalysis.calcChiefInitPosition() * ResAnalysis.VisualisationScaleFactor;
  block.Dwork(2).Data = ResAnalysis.calcDeputyInitPosition(StateVectorType) * ResAnalysis.VisualisationScaleFactor;
    
%endfunction

function Output(block)
  t = block.InputPort(1).Data;
  ResAnalysis = block.DialogPrm(1).Data; % ResultsAnalysis class object
  StateVectorType = ResAnalysis.SimulationScenario.RelMotModel.StateVectorType;
  block.OutputPort(1).Data = ...
  ResAnalysis.calcChiefPositionHistory(t) * ResAnalysis.VisualisationScaleFactor;
  block.OutputPort(2).Data = ...
  ResAnalysis.calcDeputyPositionHistory(StateVectorType, t) * ResAnalysis.VisualisationScaleFactor;
  block.OutputPort(3).Data = block.Dwork(1).Data;  
  block.OutputPort(4).Data = block.Dwork(2).Data;  
  
%endfunction

