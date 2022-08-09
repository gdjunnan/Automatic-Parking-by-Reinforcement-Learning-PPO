%% Design MPC controller for tracking

% Copyright 2020 The MathWorks, Inc.

Xref = getRefTraj(map,egoInitialPose,Ts,Tf);
pTracking = 10;
% nlobjTracking = createNLMPC(pTracking);
x = egoInitialPose';
u = [0;0];
[Ad,Bd,Cd,Dd,U0,Y0,X0,DX0] = vehicleStateJacobianFcnDT(Ts,x,u);
dsys = ss(Ad,Bd,Cd,Dd,'Ts',Ts);
mpcobj = createAdaptiveMPC(pTracking,dsys);

%% local function: Design Nonlinear MPC Tracking Controller
function nlobjTracking = createNLMPC(pTracking)
mpcverbosity('off');
nlobjTracking = nlmpc(3,3,2);
Ts = 0.1;
nlobjTracking.Ts = Ts;
nlobjTracking.PredictionHorizon = pTracking;
nlobjTracking.ControlHorizon = pTracking;
nlobjTracking.MV(1).Min = -5;
nlobjTracking.MV(1).Max = 5;
nlobjTracking.MV(2).Min = -pi/4;
nlobjTracking.MV(2).Max = pi/4;
nlobjTracking.Weights.OutputVariables = [2,2,3]; 
nlobjTracking.Weights.ManipulatedVariablesRate = [0.1,0.2];
nlobjTracking.Model.StateFcn = "parkingVehicleStateFcnRRT";
nlobjTracking.Jacobian.StateFcn = "parkingVehicleStateJacobianFcnRRT";
end

%% local function: Design Adaptive MPC Tracking Controller
function mpcobj = createAdaptiveMPC(pTracking,dsys)

mpcverbosity('off');
mpcobj = mpc(dsys);

% mpc settings
Ts = 0.1;
mpcobj.Ts = Ts;
mpcobj.PredictionHorizon = pTracking;
mpcobj.ControlHorizon = pTracking;

% mpc limits
mpcobj.MV(1).Min = -5;
mpcobj.MV(1).Max = 5;
mpcobj.MV(2).Min = -pi/4;
mpcobj.MV(2).Max = pi/4;

% mpc weights
mpcobj.Weights.OutputVariables = [2,2,3];
mpcobj.Weights.ManipulatedVariablesRate = [0.1,0.2];
% mpcobj.Weights.MV = [0, 0.01];

% custom estimator 
setoutdist(mpcobj,'model',tf(ones(3,1)));
setEstimator(mpcobj,'custom');
end