function DiscControllerSFcn(block)
% Controller S-Function
% Level-2 MATLAB file S-Function.

  setup(block);
  
%endfunction

function setup(block)
 
  block.NumDialogPrms  = 1;
  
  SimScenario = block.DialogPrm(1).Data; % Simulation class object 
  
  %% Register number of input and output ports
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 6;
  block.InputPort(1).DirectFeedthrough = false;
  
  block.InputPort(2).Dimensions        = 1;
  block.InputPort(2).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 3;
  
  %% Set block sample time
  Ts = SimScenario.Ts;
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
  block.NumDworks = 3;
  
  block.Dwork(1).Name = 'x_hat'; % Current state estimate
  block.Dwork(1).Dimensions      = 7;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name = 'X_hat'; % Current augmented state estimate
  block.Dwork(2).Dimensions      = 14;
  block.Dwork(2).DatatypeID      = 0;
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;
    
  block.Dwork(3).Name = 'ukm1'; % Last control move
  block.Dwork(3).Dimensions      = 3;
  block.Dwork(3).DatatypeID      = 0;
  block.Dwork(3).Complexity      = 'Real';
  block.Dwork(3).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
  SimScenario = block.DialogPrm(1).Data; % Simulation class object
  block.Dwork(1).Data = SimScenario.Controller.calcInitialStateEstimate(); % Initial state estimate
  block.Dwork(2).Data = SimScenario.Controller.calcInitialAugmentedStateEstimate();
  block.Dwork(3).Data = SimScenario.Controller.u0; % Initial control value
  
%endfunction

function Output(block)
  SimScenario = block.DialogPrm(1).Data; % Simulation class object
  t = block.CurrentTime;
  x_hat = block.Dwork(1).Data; % Current state estimate
  X_hat = block.Dwork(2).Data; % Current augmented state estimate
  ukm1 = block.Dwork(3).Data; % Previous control move
  
  u = SimScenario.Controller.calcControllerMove(x_hat, X_hat, ukm1, t); % u(k)
  block.Dwork(3).Data = u; % Storing u(k)  
  block.OutputPort(1).Data = u;
  
  ym_rm = block.InputPort(1).Data;
  ym_ep = block.InputPort(2).Data;
  ym = [ym_rm; ym_ep];
  
  x_hat_kp1 = SimScenario.Controller.calcStateEstimate(ym, x_hat, u, t);
  block.Dwork(1).Data = x_hat_kp1;
  block.Dwork(2).Data = SimScenario.Controller.calcAugmentedStateEstimate(x_hat_kp1, x_hat, u, t);
  
%endfunction

