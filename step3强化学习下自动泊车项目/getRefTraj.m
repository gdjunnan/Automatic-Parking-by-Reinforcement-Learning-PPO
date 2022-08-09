function Xref = getRefTraj(map,egoInitialPose,Ts,Tf)
% convert the [x, y, theta] path obtained from the map into a time based
% trajectory for reference tracking.

% Copyright 2020 The MathWorks, Inc.
    [xref0,yref0,tref0] = getReferencePath(map);
    dist = (xref0-egoInitialPose(1)).^2 + (yref0-egoInitialPose(2)).^2 + 10*(tref0-egoInitialPose(3)).^2;
    [~,startIdx] = min(dist);
    xrefReal = [xref0(startIdx:end),xref0(1:end),xref0(1:end)];
    yrefReal = [yref0(startIdx:end),yref0(1:end),yref0(1:end)];
    trefReal = [tref0(startIdx:end),2*pi+tref0(1:end),4*pi+tref0(1:end)];
    Tsteps = Tf/Ts;
    xRef = [xrefReal;yrefReal;trefReal]';
    p = size(xRef,1);
    Xref = [xRef(1:p,:);repmat(xRef(end,:),Tsteps-p,1)];
end