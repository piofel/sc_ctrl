function DiscInDisturbanceSFcn(block)
% Level-2 MATLAB file S-Function.

  setup(block);
  
%endfunction

function setup(block)

  block.NumDialogPrms = 1;

  SimScenario = block.DialogPrm(1).Data; % Simulation class object 
  
  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
  
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(1).SamplingMode = 'Sample';
  
  block.OutputPort(1).Dimensions = 3;
  block.OutputPort(1).SamplingMode = 'Sample';
  
  %% Set block sample time
  Ts = SimScenario.DistModel.distTs;
  block.SampleTimes = [Ts 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';
  
  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions); 
  block.RegBlockMethod('Outputs',                 @Output);  
  
%endfunction

function DoPostPropSetup(block)

  %% Setup Dwork
  block.NumDworks = 0;

%endfunction

function InitConditions(block)
block.OutputPort(1).Data = zeros(3,1);
    
%endfunction

function Output(block)
  SimScenario = block.DialogPrm(1).Data; % Simulation class object
  DeputyMass = block.InputPort(1).Data;
  block.OutputPort(1).Data = SimScenario.DistModel.calcInputDisturbance(DeputyMass);
  
%endfunction

