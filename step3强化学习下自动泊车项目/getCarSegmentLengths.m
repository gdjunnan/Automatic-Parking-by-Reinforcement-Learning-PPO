function c = getCarSegmentLengths(L,W,angles)
% get distances from center of car to car boundary given angles, length and
% width of car

% Copyright 2020 The MathWorks, Inc.

%# codegen

% make col vector and wrap angles
angles = angles(:);
angles = atan2(sin(angles),cos(angles));

% get some lengths
C = norm([L,W])/2;
L = L/2; W = W/2;

%% 1
idx = cos(angles)*C >= L ;
c1a = localGetLengthsForRegion(L,angles(idx & angles >= 0));
c1b = localGetLengthsForRegion(L,angles(idx & angles <  0));

%% 2
idx = sin(angles)*C > W;
c2 = localGetLengthsForRegion(W,angles(idx) - pi/2);

%% 3
idx = cos(angles)*C <= -L;
c3 = localGetLengthsForRegion(L,angles(idx) - pi);

%% 4
idx = sin(angles)*C < -W;
c4 = localGetLengthsForRegion(W,angles(idx) - pi*3/2);

%%
c = [c1a;c2;c3;c4;c1b];

function c = localGetLengthsForRegion(x,angles)
c = x./cos(angles);

