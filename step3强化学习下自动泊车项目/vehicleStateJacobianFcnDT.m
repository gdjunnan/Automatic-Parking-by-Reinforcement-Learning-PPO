function [Ad,Bd,Cd,Dd,U,Y,X,DX] = vehicleStateJacobianFcnDT(Ts, x, u) %#codegen
% Jacobian of model equations.
% state variables x, y and yaw angle theta.
% control variables v and steering angle delta.

% Copyright 2020 The MathWorks, Inc.

%% continuous time model
% Parameters
wb = 2.8;

% Variables
theta = x(3);
v = u(1);
delta = u(2);

% Linearize the state equations at the current condition
A = zeros(3,3);
B = zeros(3,2);

A(1,3) = -v*sin(theta);
B(1,1) = cos(theta);

A(2,3) = v*cos(theta);
B(2,1) = sin(theta);

B(3,1) = 1/wb*tan(delta);
B(3,2) = 1/wb*(v*(1+tan(delta)^2));

C = eye(3);
D = zeros(3,2);

%% Discretize CT model and calculate DX for DT model
nu = 2;
nx = 3;
dxdt = vehicleStateFcn(x, u);
M = expm([[A B dxdt]*Ts; zeros(nu+1,nx+nu+1)]);
Ad = M(1:nx,1:nx);
Bd = M(1:nx,nx+(1:nu));
Cd = C;
Dd = D;
X = x;
U = u;
Y = x;
DX = M(1:nx,end); 
