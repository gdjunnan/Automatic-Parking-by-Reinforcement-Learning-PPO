classdef ParkingLot < handle
% MAP = ParkingLot(FREEIDXS) creates a parking lot with the free spots 
% specified in the array FREEIDXS.
% 
% MAP is a struct containing the following fields:
%   1. XLimits - map X limits
%   2. YLimits - map Y limits
%   3. SpotLocations = [x y] of occupied spaces
%   4. OccupiedSpots = binary array of size 1x64, 1 occupied, 0 free
%   5. TotalSpots - total number of parking spots
%   6. AxesHandle - axes handle of the figure
%
% Total number of parking spots in this parking lot is 64 (non-tunable).

% Copyright 2020 The MathWorks, Inc.

    properties
        VehiclePose         = [20 16 0]
        TrainingZoneXLimits = [36.5 59]
        TrainingZoneYLimits = [0 20]
    end
    properties (SetAccess = private)
        XLimits             = [0 100]
        YLimits             = [0 29.9]
        SpotDimensions      = [4.5 7]
        ObstacleDimensions  = [4.7 2.5]
        VehicleDimensions   = [4.7 1.8]
        Wheelbase           = 2.8
        PassageWidth        = 16
        MaxSpots            = 64
        SpotLocations       = zeros(64,2)
        SensorLocations     = zeros(64,2)
        OccupiedSpots       = ones(1,64)
        ObstacleMatrix
    end
    properties (Hidden)
        Figure
        VehicleBodyHandle
        VehicleAxlesHandle
        VehicleWheelsHandle
        LidarLinesHandle
        LidarCrossHandle
        CameraFOVHandle
        TrainZoneHandle
        VehicleStatusHandle
        ControllerStatusHandle
        TimeDisplayHandle
        ReferencePathRect
        ReferencePathCurvature = 0.3
    end
    methods
        function map = ParkingLot(freeidxs)
            arguments
                freeidxs (1,1) double {localValidateIdx}
            end
            
            map.OccupiedSpots(freeidxs) = 0;
            
            map.ReferencePathRect = [ map.XLimits(1)+map.SpotDimensions(2)+0.5*map.PassageWidth, ...
                                      map.YLimits(1)+map.SpotDimensions(2)+0.5*map.PassageWidth, ...
                                      diff(map.XLimits)-2*(map.SpotDimensions(2)+0.5*map.PassageWidth), ...
                                      diff(map.YLimits)-2*(map.SpotDimensions(2)+0.5*map.PassageWidth) ];
            
            buildFigure(map);
            initializeMap(map);
            
            map.ObstacleMatrix = getObstacles(map, find(map.OccupiedSpots==1));
            
            map.Figure.Visible = 'on';
        end
        function show(map)
        % show(map) displays the map in a figure.
            if isempty(map.Figure)
                updateMap(map);
            else
                map.Figure.Visible = 'on';
            end
        end
        function buildFigure(map)
            % Build the figure
            map.Figure             = figure;
            map.Figure.Position    = [435 200 1290 740];
            map.Figure.Name        = 'Auto Parking Valet';
            map.Figure.NumberTitle = 'off';
            %map.Figure.MenuBar = 'none';
            %map.Figure.CloseRequestFcn = @localHideFigure;
            ax = gca(map.Figure);
            legend(ax,'off');
            hold(ax, 'on');
            ax.XLimMode = 'manual';
            ax.YLimMode = 'manual';
            ax.XLim = map.XLimits;
            ax.YLim = map.YLimits;
        end
        function highlight(map,idx)
        % highlight(map,idx) highlights a spot in the map specified in idx.
            arguments
                map
                idx (1,1) double
            end
            ax = gca(map.Figure);
            iseastspot = idx >= 15 && idx <= 22;
            iswestspot = idx >= 37 && idx <= 40;
            if iseastspot || iswestspot
                posx = map.SpotLocations(idx,1) - 0.5*map.ObstacleDimensions(1) - 0.5;
                posy = map.SpotLocations(idx,2) - 0.5*map.ObstacleDimensions(2) - 0.5;
                len = map.ObstacleDimensions(1) + 1;
                wid = map.ObstacleDimensions(2) + 1;
            else
                posx = map.SpotLocations(idx,1) - 0.5*map.ObstacleDimensions(2) - 0.5;
                posy = map.SpotLocations(idx,2) - 0.5*map.ObstacleDimensions(1) - 0.5;
                len = map.ObstacleDimensions(2) + 1;
                wid = map.ObstacleDimensions(1) + 1;
            end
            hlight = findobj(ax,'Tag','hlight');
            if isempty(hlight)
                rectangle(ax,'Position',[posx posy len wid],'EdgeColor','k','LineWidth',2,'Tag','hlight') 
            else
                hlight.Position = [posx posy len wid];
            end
            drawnow;
        end
        function setFree(map,idxs)
        % setFree(map,idxs) frees the spots with index IDXS.
            localValidateIdx(idxs)
            map.OccupiedSpots(idxs) = 0;
            map.ObstacleMatrix = getObstacles(map, find(map.OccupiedSpots==1));
            initializeMap(map);
        end
        function setOccupied(map,idxs)
        % setOccupied(map,idxs) occupies the spots with index IDXS.
            localValidateIdx(idxs)
            map.OccupiedSpots(idxs) = 1;
            map.ObstacleMatrix = getObstacles(map, find(map.OccupiedSpots==1));
            initializeMap(map);
        end
        function obs = getObstacles(map,spotidx)
        % obs = getObstacles(map) returns the list of obstacles in the
        % parking lot in [x y xlen ylen theta] format.
        %
        % obs = getObstacles(map,spotidx) returns the obstacle at the
        % spot index spotidx in [x y xlen ylen theta] format.
            arguments
                map
                spotidx (1,:) double = find(map.OccupiedSpots)
            end
            obs = zeros(numel(spotidx)+11,5);
            ct = 1;
            while ct <= numel(spotidx)
                iseastspot = spotidx(ct) >= 15 && spotidx(ct) <= 22;
                iswestspot = spotidx(ct) >= 37 && spotidx(ct) <= 40;
                if iseastspot || iswestspot
                    obs(ct,1) = map.SpotLocations(spotidx(ct),1);
                    obs(ct,2) = map.SpotLocations(spotidx(ct),2);
                    obs(ct,3) = map.ObstacleDimensions(1);
                    obs(ct,4) = map.ObstacleDimensions(2);
                    obs(ct,5) = 0;
                else
                    obs(ct,1) = map.SpotLocations(spotidx(ct),1);
                    obs(ct,2) = map.SpotLocations(spotidx(ct),2);
                    obs(ct,3) = map.ObstacleDimensions(2);
                    obs(ct,4) = map.ObstacleDimensions(1);
                    obs(ct,5) = 0;
                end
                ct = ct + 1;
            end
            % add obstacles at corners
            obs(ct:ct+5,:) = [9.25 3.35 16.5 4.7 0; ...
                              90.75 3.35 16.5 4.7 0; ...
                              96.35 8.5 4.7 5 0; ...
                              96.35 51.5 4.7 5 0; ...
                              90.75 56.35 16.5 4.7 0; ...
                              9.25 56.35 16.5 4.7 0];
            % add obstacles at map boundary
            obs(ct+6:end,:) = ...
                [50 0.25 100 0.5 0; ...
                 99.75 30 0.5 60 0; ...
                 50 59.75 100 0.5 0; ...
                 0.25 30 0.5 60 0; ...
                 50 30 54 0.5 0];
        end
        function pose = createTargetPose(map,idx)
        % pose = createTargetPose(map,idx,wbase,theta) returns a target
        % pose for the vehicle based on the specified parking spot index
        % idx and wheelbase length wbase.
            arguments
                map
                idx (1,1) double {mustBePositive,mustBeInteger}
            end
            wbase = map.Wheelbase;
            loc = map.SpotLocations(idx,:);
            issouthspot = idx <= 14;
            iseastspot = idx >= 15 && idx <= 22;
            isnorthspot = idx >= 23 && idx <= 36;
            iswestspot = idx >= 37 && idx <= 40; %#ok<NASGU>
            if issouthspot
                pose(1) = loc(1);
                pose(2) = loc(2) + 0.5*wbase;
                pose(3) = -pi/2;
            elseif iseastspot
                pose(1) = loc(1) - 0.5*wbase;
                pose(2) = loc(2);
                pose(3) = 0;
            elseif isnorthspot
                pose(1) = loc(1);
                pose(2) = loc(2) - 0.5*wbase;
                pose(3) = pi/2;
            else
                pose(1) = loc(1) + 0.5*wbase;
                pose(2) = loc(2);
                pose(3) = pi;
            end
        end
        function [xref,yref,tref] = getReferencePath(map)
        % [xref,yref,tref] = getReferencePath(map) returns the x, y and
        % theta reference path that a vehicle can follow while searching
        % for a parking spot.
            x0 = map.ReferencePathRect(1);
            y0 = map.ReferencePathRect(2);
            w  = map.ReferencePathRect(3);
            h  = map.ReferencePathRect(4);
            r  = map.ReferencePathCurvature;
            d  = min(w,h);
            nlarge = 120;
            nsmall = 50;
            ncurve = 30;
            % start from straight section (south)
            xstartS = x0 + 0.5*r*d;
            xendS   = x0 + w - 0.5*r*d;
            yS      = y0;
            xrefS   = linspace(xstartS,xendS,nlarge);   
            yrefS   = yS * ones(1,nlarge);
            trefS   = zeros(1,nlarge);
            % lower curve (east)
            phi = linspace(-pi/2,0,ncurve);
            xstartC1E = xendS;
            yendC1E   = yS + 0.5*r*d;
            xrefC1E   = xstartC1E + 0.5*r*d*cos(phi);
            yrefC1E   = yendC1E   + 0.5*r*d*sin(phi);
            trefC1E   = pi/2 + phi;
            % straight section (east)
            xE      = x0 + w;
            ystartE = y0 + 0.5*r*d;
            yendE   = y0 + h - 0.5*r*d;
            xrefE   = xE * ones(1,nsmall);
            yrefE   = linspace(ystartE,yendE,nsmall);
            trefE   = pi/2 * ones(1,nsmall);
            % upper curve (east)
            phi = linspace(0,pi/2,ncurve);
            xendC2E   = xendS;
            ystartC2E = yendE;
            xrefC2E   = xendC2E   + 0.5*r*d*cos(phi);
            yrefC2E   = ystartC2E + 0.5*r*d*sin(phi);
            trefC2E   = pi/2 + phi;
            % straight section (north)
            xstartN = xendC2E;
            xendN   = xstartS;
            yN      = y0 + h;
            xrefN   = linspace(xstartN,xendN,nlarge);
            yrefN   = yN * ones(1,nlarge);
            trefN   = pi * ones(1,nlarge);
            % upper curve (west)
            phi = linspace(pi/2,pi,ncurve+10);
            xstartC1W = xstartS;
            yendC1W   = ystartC2E;
            xrefC1W   = xstartC1W + 0.5*r*d*cos(phi);
            yrefC1W   = yendC1W   + 0.5*r*d*sin(phi);
            trefC1W   = pi/2 + phi;
            % straight section (west)
            xW      = x0;
            ystartW = yendC1W;
            yendW   = ystartE;
            xrefW   = xW * ones(1,nsmall);
            yrefW   = linspace(ystartW,yendW,nsmall);
            trefW   = 3*pi/2 * ones(1,nsmall);
            % lower curve (west)
            phi = linspace(pi,3*pi/2,ncurve);
            xendC2W   = xstartS;
            ystartC2W = yendW;
            xrefC2W   = xendC2W   + 0.5*r*d*cos(phi);
            yrefC2W   = ystartC2W + 0.5*r*d*sin(phi);
            trefC2W   = pi/2 + phi;
            xref = [xrefS xrefC1E xrefE xrefC2E xrefN xrefC1W xrefW xrefC2W];
            yref = [yrefS yrefC1E yrefE yrefC2E yrefN yrefC1W yrefW yrefC2W];
            tref = [trefS trefC1E trefE trefC2E trefN trefC1W trefW trefC2W];
        end
        function ax = getAxesHandle(map)
        % getAxesHandle(map) returns the axes handle to the figure.
            ax = gca(map.Figure);
        end
        function delete(map)
        % delete(map) deletes the figure.
            if ~isempty(map.Figure) || ~isvalid(map.Figure)
                delete(map.Figure);
            end
        end
    end
    methods (Access = private)
        function initializeMap(map)
            
            if isempty(map.Figure) || ~isvalid(map.Figure)
                buildFigure(map);
            end
            
            % get handle and clear axis
            ax = gca(map.Figure);
            if ~isempty(map.Figure)
                cla(ax)
            end
            map.Figure.Visible = 'on';
           %轴
            axis(ax,'equal')
            axis(ax,'tight')
            
            % Number of spots in N,S,E,W
            numspotsnorth = 14;
            numspotssouth = 14;
            numspotseast  = 8;
            numspotswest  = 4;
            xstartnorth = 18.5;
            xstartsouth = 18.5;
            ystarteast  = 12;
            ystartwest  = 21;            
            spotIdx = 1;
            txtoffset = 0.5*map.SpotDimensions(2);

            rectangle(ax,'Position',[map.XLimits(1) map.YLimits(1) map.XLimits(2) map.YLimits(2)],'LineWidth',2)
            
   
            % Plot south spots
            for i = 1:numspotssouth+1
                % Plot parking lines
                x = (xstartsouth + (i-1)*map.SpotDimensions(1))*[1 1];
                y = map.YLimits(1) + [0 map.SpotDimensions(2)];
                line(ax,x,y,'LineWidth',2 ,'color','k')
                
                if i<=numspotssouth  % to avoid plotting an extra vehicle at the end
                    % Plot spot number
                    %text(ax,x(1)+0.5*map.SpotDimensions(1)-1,y(1)+0.5*diff(y)+txtoffset,sprintf('%02u',spotIdx))
                    
                    map.SpotLocations(spotIdx,:) = [x(1)+0.5*map.SpotDimensions(1) y(1)+0.5*diff(y)];
                    obsx = map.SpotLocations(spotIdx,1) - 0.5*map.ObstacleDimensions(2);
                    obsy = map.SpotLocations(spotIdx,2) - 0.5*map.ObstacleDimensions(1);
%                     obsxx = map.SpotLocations(spotIdx,1) + 0.1*map.ObstacleDimensions(2);
%                     obsyy = map.SpotLocations(spotIdx,2) + 0.1*map.ObstacleDimensions(1);
                    sensorRectX = obsx + 0.5*map.ObstacleDimensions(2) - 0.5;
                    sensorRectY = map.YLimits(1) + map.SpotDimensions(2) + 2 - 0.5;
                    map.SensorLocations(spotIdx,:) = [sensorRectX+0.5 sensorRectY+0.5];
                    
                    if map.OccupiedSpots(spotIdx)
                        
                        % Plot occupied spots
                        rectangle(ax,'Position',[obsx obsy map.ObstacleDimensions(2) map.ObstacleDimensions(1)],'FaceColor','r') % color of other cars
                        % Plot sensor rectangles in red    %传感器矩形                    
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','r')
                        map.OccupiedSpots(spotIdx) = 1;
                    else
                        % Plot sensor rectangles in green                        
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','g')
                    end            
                                        spotIdx = spotIdx + 1;
                end
            end

            % Plot east spots
            for i = 1:numspotseast+1
                % parking lines
                x = map.XLimits(2) + [-map.SpotDimensions(2) 0];
                y = (ystarteast + (i-1)*map.SpotDimensions(1))*[1 1];
                %line(ax,x,y,'LineWidth',2)
                
                if i<=numspotseast
                    % Plot spot number
                    %数字
                    %text(ax,x(1)+0.5*diff(x)-1-txtoffset,y(1)+0.5*map.SpotDimensions(1),sprintf('%02u',spotIdx))
                    
                    map.SpotLocations(spotIdx,:) = [x(1)+0.5*diff(x) y(1)+0.5*map.SpotDimensions(1)];
                    obsx = map.SpotLocations(spotIdx,1) - 0.5*map.ObstacleDimensions(1);
                    obsy = map.SpotLocations(spotIdx,2) - 0.5*map.ObstacleDimensions(2);
                    sensorRectX = map.XLimits(2) - map.SpotDimensions(2) - 2.5;
                    sensorRectY = obsy + 0.5*map.ObstacleDimensions(2) - 0.5;
                    map.SensorLocations(spotIdx,:) = [sensorRectX+0.5 sensorRectY+0.5];
                    
                    if map.OccupiedSpots(spotIdx)
                     %同上
                        % Plot occupied spots
                        %rectangle(ax,'Position',[obsx obsy map.ObstacleDimensions(1) map.ObstacleDimensions(2)],'FaceColor','k')
                        % Plot sensor rectangles in red                        
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','r')
                        map.OccupiedSpots(spotIdx) = 1;
                    else
                        % Plot sensor rectangles in green                        
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','g')
                    end 
                    
                    spotIdx = spotIdx + 1;
                end
            end

            % Plot north spots
            for i = numspotsnorth+1:-1:1
                % parking lines
                x = (xstartnorth + (i-1)*map.SpotDimensions(1))*[1 1];
                y = map.YLimits(2) + [-map.SpotDimensions(2) 0];
                %line(ax,x,y,'LineWidth',2)
                
                if i<=numspotsnorth
                    % Plot spot number
                    %text(ax,x(1)+0.5*map.SpotDimensions(1)-1,y(1)+0.5*diff(y)-txtoffset,sprintf('%02u',spotIdx))
                    
                    map.SpotLocations(spotIdx,:) = [x(1)+0.5*map.SpotDimensions(1) y(1)+0.5*diff(y)];
                    obsx = map.SpotLocations(spotIdx,1) - 0.5*map.ObstacleDimensions(2);
                    obsy = map.SpotLocations(spotIdx,2) - 0.5*map.ObstacleDimensions(1);
                    sensorRectX = obsx + 0.5*map.ObstacleDimensions(2) - 0.5;
                    sensorRectY = map.YLimits(2) - map.SpotDimensions(2) - 2.5;
                    map.SensorLocations(spotIdx,:) = [sensorRectX+0.5 sensorRectY+0.5];
                    
                    if map.OccupiedSpots(spotIdx)
                        % Plot occupied spots
                        %rectangle(ax,'Position',[obsx obsy map.ObstacleDimensions(2) map.ObstacleDimensions(1)],'FaceColor','k')
                        % Plot sensor rectangles in red                        
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','r')
                        map.OccupiedSpots(spotIdx) = 1;
                    else
                        % Plot sensor rectangles in green                        
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','g')
                    end 
                    
                    spotIdx = spotIdx + 1;
                end
            end

            % Plot west spots
            for i = numspotswest+1:-1:1
                % parking lines
                x = map.XLimits(1) + [0 map.SpotDimensions(2)];
                y = (ystartwest + (i-1)*map.SpotDimensions(1))*[1 1];
               % line(ax,x,y,'LineWidth',2)
                
                if i<=numspotswest
                    % Plot spot number
                    %text(ax,x(1)+0.5*diff(x)-1+txtoffset,y(1)+0.5*map.SpotDimensions(1),sprintf('%02u',spotIdx))
                    
                    map.SpotLocations(spotIdx,:) = [x(1)+0.5*diff(x) y(1)+0.5*map.SpotDimensions(1)];
                    obsx = map.SpotLocations(spotIdx,1) - 0.5*map.ObstacleDimensions(1);
                    obsy = map.SpotLocations(spotIdx,2) - 0.5*map.ObstacleDimensions(2);
                    sensorRectX = map.XLimits(1) + map.SpotDimensions(2) + 1.5;
                    sensorRectY = obsy + 0.5*map.ObstacleDimensions(2) - 0.5;
                    map.SensorLocations(spotIdx,:) = [sensorRectX+0.5 sensorRectY+0.5];
                    
                    if map.OccupiedSpots(spotIdx)
                        % Plot occupied spots
                        %rectangle(ax,'Position',[obsx obsy map.ObstacleDimensions(1) map.ObstacleDimensions(2)],'FaceColor','k')
                        % Plot sensor rectangles in red                        
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','r')
                        map.OccupiedSpots(spotIdx) = 1;
                    else
                        % Plot sensor rectangles in green                        
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','g')
                    end
                    
                    spotIdx = spotIdx + 1;
                end
            end

            % Plot middle spots (south)
            xstartmid = map.SpotDimensions(2) + map.PassageWidth;
            ystartmid = map.SpotDimensions(2) + map.PassageWidth;
            midcols = (diff(map.XLimits) - 2*(map.SpotDimensions(2)+map.PassageWidth))/map.SpotDimensions(1);
            line(ax,xstartmid+[0 midcols*map.SpotDimensions(1)],(ystartmid+map.SpotDimensions(2))*[1 1],'LineWidth',2)
            for i = 1:midcols+1
                % parking lines
                x = (xstartmid + (i-1)*map.SpotDimensions(1))*[1 1];
                y = ystartmid + [0 map.SpotDimensions(2)];
                %line(ax,x,y,'LineWidth',2)
                
                if i<=midcols
                    % Plot spot number
                    %text(ax,x(1)+0.5*map.SpotDimensions(1)-1,y(1)+0.5*diff(y)-txtoffset,sprintf('%02u',spotIdx))
                    
                    map.SpotLocations(spotIdx,:) = [x(1)+0.5*map.SpotDimensions(1) y(1)+0.5*diff(y)];
                    obsx = map.SpotLocations(spotIdx,1) - 0.5*map.ObstacleDimensions(2);
                    obsy = map.SpotLocations(spotIdx,2) - 0.5*map.ObstacleDimensions(1);
                    sensorRectX = obsx + 0.5*map.ObstacleDimensions(2) - 0.5;
                    sensorRectY = obsy - 3.5;
                    map.SensorLocations(spotIdx,:) = [sensorRectX+0.5 sensorRectY+0.5];
                    
                    if map.OccupiedSpots(spotIdx)
                        % Plot occupied spots
                        %rectangle(ax,'Position',[obsx obsy map.ObstacleDimensions(2) map.ObstacleDimensions(1)],'FaceColor','k')
                        % Plot sensor rectangles in red                        
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','r')
                        map.OccupiedSpots(spotIdx) = 1;
                    else
                        % Plot sensor rectangles in green                        
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','g')
                    end
                    
                    spotIdx = spotIdx + 1;
                end
            end

            % Plot middle spots (north)
            ystartmid = 2*map.SpotDimensions(2) + map.PassageWidth;
            for i = midcols+1:-1:1
                % parking lines
                x = (xstartmid + (i-1)*map.SpotDimensions(1))*[1 1];
                y = ystartmid + [0 map.SpotDimensions(2)];
                %line(ax,x,y,'LineWidth',2)
                
                if i<=midcols
                    % Plot spot number
                    %text(ax,x(1)+0.5*map.SpotDimensions(1)-1,y(1)+0.5*diff(y)+txtoffset,sprintf('%02u',spotIdx))
                    
                    map.SpotLocations(spotIdx,:) = [x(1)+0.5*map.SpotDimensions(1) y(1)+0.5*diff(y)];
                    obsx = map.SpotLocations(spotIdx,1) - 0.5*map.ObstacleDimensions(2);
                    obsy = map.SpotLocations(spotIdx,2) - 0.5*map.ObstacleDimensions(1);
                    sensorRectX = obsx + 0.5*map.ObstacleDimensions(2) - 0.5;
                    sensorRectY = obsy + map.ObstacleDimensions(1) + 2.5;
                    map.SensorLocations(spotIdx,:) = [sensorRectX+0.5 sensorRectY+0.5];
                    
                    if map.OccupiedSpots(spotIdx)
                        % Plot occupied spots
                        %rectangle(ax,'Position',[obsx obsy map.ObstacleDimensions(2) map.ObstacleDimensions(1)],'FaceColor','k')
                        % Plot sensor rectangles in red                        
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','r')
                        map.OccupiedSpots(spotIdx) = 1;
                    else
                        % Plot sensor rectangles in green                        
                        %rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','g')
                    end
                    
                    spotIdx = spotIdx + 1;
                end
            end
            
            % plot corner obstacles
            color = [192 192 192]/255; %[240 128 128]/255;  % color of no parking
            pg1 = polyshape([1 17.5 17.5 5 5 94 94 82.5 82.5 99 99 1],[1 1 5.7 5.7 24 24 5.7 5.7 1 1 29 29]);
            plot(ax,pg1,'FaceColor',color,'EdgeColor',color);
%             pg1 = polyshape([1 17.5 17.5 1],[1 1 5.7 5.7]);
%             plot(ax,pg1,'FaceColor',color,'EdgeColor',color);
%             pg2 = polyshape([82.5 99 99 82.5],[1 1 6 6]);
%             plot(ax,pg2,'FaceColor',color,'EdgeColor',color);
%             pg3 = polyshape([94 99 99 82.5 82.5 94],[49 49 59 59 54 54]);
%             plot(ax,pg3,'FaceColor',color,'EdgeColor',color);
%             pg4 = polyshape([1 17.5 17.5 1],[54 54 58.7 58.7]);
%             plot(ax,pg4,'FaceColor',color,'EdgeColor',color);
              text(ax,42,26.5,'NO PARKING','Color','r','FontWeight','bold','FontSize',20);
              text(ax,25,22,'Automatic Parking based on Reinforcement Learning from SDU','Color','r','FontWeight','bold','FontSize',12);
              text(ax,1.2,28.7,'The code is modified from the Train PPO Agent for Automatic Parking Valet in MathWorks','Color','b','FontWeight','bold','FontSize',8);
%             text(ax,86,56.5,'NO PARKING','Color','w','FontWeight','bold');
            
            ax.XLim = map.XLimits;
            ax.YLim = map.YLimits;
        end
    end
    
    methods        
        function plotVehicle(map,pose,steer)
            ax =  gca(map.Figure);
            
            % extract data and translate pose to center of vehicle
            th = pose(3);
            xc = pose(1) + 0.5*map.Wheelbase*cos(th);
            yc = pose(2) + 0.5*map.Wheelbase*sin(th);
            
            % corners of the vehicle rectangle inthe order LU LD RD RU
            cornersx = xc + 0.5*map.VehicleDimensions(1)*[-1 -1 1 1];
            cornersy = yc + 0.75*map.VehicleDimensions(2)*[1 -1 -1 1];
            vbody = rotate( polyshape(cornersx,cornersy), rad2deg(th), [xc yc] );
            
            % prepare axles
            axwid = map.VehicleDimensions(1)/25; % axle width
            faxlex = xc + 0.5*map.Wheelbase*[1 1 1 1]+ 0.5*axwid*[-1 -1 1 1];
            raxlex = xc + 0.5*map.Wheelbase*[-1 -1 -1 -1]+ 0.5*axwid*[-1 -1 1 1];
            faxley = yc + 0.7*map.VehicleDimensions(2)*[1 -1 -1 1];
            raxley = faxley;
            axles(1) = rotate( polyshape(faxlex,faxley), rad2deg(th), [xc yc] );
            axles(2) = rotate( polyshape(raxlex,raxley), rad2deg(th), [xc yc] );
            maxlex = xc + 7*axwid*[-1 -1 1 1];
            maxley = yc + 0.15*map.VehicleDimensions(2)*[1 -1 -1 1];
            axles(3) = rotate( polyshape(maxlex,maxley), rad2deg(th), [xc yc] );
            
            % prepare wheels
            whlen = map.VehicleDimensions(1)/5;  % wheel rectangle length
            whwid = map.VehicleDimensions(1)/10; % wheel rectangle width
            whx = 0.5*whlen*[-1 -1 1 1]; % wheel rectangle corners x
            why = 0.5*whwid*[1 -1 -1 1]; % wheel rectangle corners y
            wheels0(1) = rotate( polyshape(xc-0.5*map.Wheelbase+whx, yc+0.7*map.VehicleDimensions(2)+why), rad2deg(th), [xc yc] );
            wheels0(2) = rotate( polyshape(xc-0.5*map.Wheelbase+whx, yc-0.7*map.VehicleDimensions(2)+why), rad2deg(th), [xc yc] );
            wheels0(3) = rotate( polyshape(xc+0.5*map.Wheelbase+whx, yc-0.7*map.VehicleDimensions(2)+why), rad2deg(th), [xc yc] );
            wheels0(4) = rotate( polyshape(xc+0.5*map.Wheelbase+whx, yc+0.7*map.VehicleDimensions(2)+why), rad2deg(th), [xc yc] );
%             wheels(1) = wheels0(1);
%             wheels(2) = wheels0(2);
%             wheels(3) = rotate( wheels0(3), rad2deg(steer), [axles(1).Vertices(3,1), axles(1).Vertices(3,2)+0.5*axwid] );
%             wheels(4) = rotate( wheels0(4), rad2deg(steer), [axles(1).Vertices(2,1), axles(1).Vertices(2,2)+0.5*axwid] );
            wheels(1) = rotate( wheels0(1), rad2deg(steer), [axles(2).Vertices(2,1), axles(2).Vertices(2,2)+0.5*axwid] );
            wheels(2) = rotate( wheels0(2), rad2deg(steer), [axles(2).Vertices(3,1), axles(2).Vertices(3,2)+0.5*axwid] );
            wheels(3) = wheels0(3);
            wheels(4) = wheels0(4);
            
            if isempty(map.VehicleBodyHandle) || any(~isvalid(map.VehicleBodyHandle))
                %vehicle color
                map.VehicleBodyHandle = plot(ax,vbody,'FaceColor','b','FaceAlpha',0.25); %body
                map.VehicleAxlesHandle = plot(ax,axles,'FaceColor','g','FaceAlpha',1);   %axle
                map.VehicleWheelsHandle = plot(ax,wheels,'FaceColor','c','FaceAlpha',1); %wheel
            else
                map.VehicleBodyHandle.Shape = vbody;
                for i = 1:3
                    map.VehicleAxlesHandle(i).Shape = axles(i);
                end
                for i = 1:4
                    map.VehicleWheelsHandle(i).Shape = wheels(i);
                end
            end
        end
        
        function plotCameraFOV(map,pose,depth,viewAngle)
            ax = gca(map.Figure);
            th = pose(3);
            xc = pose(1) + 0.5*map.Wheelbase*cos(th);
            yc = pose(2) + 0.5*map.Wheelbase*sin(th);
            theta = mod(pose(3), pi);  % direction of CameraFOVHandle
            
            phi_min = theta - 0.5*viewAngle;
            phi_max = theta + 0.5*viewAngle;
            phi = linspace(phi_min,phi_max,100); % phi is in radians
            x = xc + depth * cos(0.5*phi);
            y = yc + depth * sin(phi);
            x = [x xc x(1)];
            y = [y yc y(1)];
            if isempty(map.CameraFOVHandle) || ~isvalid(map.CameraFOVHandle)
                map.CameraFOVHandle = fill(ax, x, y, 'y', 'FaceAlpha', 0.3, 'Tag', 'arc');  %camera's color
            else
                map.CameraFOVHandle.XData = x;
                map.CameraFOVHandle.YData = y;
            end
        end
        
        function plotLidar(map,pose,lidar,maxLidarDist)
            ax = gca(map.Figure);
            numSensors = numel(lidar);
            th = pose(3);
            xc = pose(1) + 0.5*map.Wheelbase*cos(th);
            yc = pose(2) + 0.5*map.Wheelbase*sin(th);
            phi    = 0:2*pi/numSensors:2*pi*(1-1/numSensors);
            
            lidar = reshape(lidar,numSensors,1);
            lidarx = xc + lidar' .* cos(th+phi);
            lidary = yc + lidar' .* sin(th+phi);
            nlines = numSensors/2;
            xplot = zeros(2,nlines);
            yplot = zeros(2,nlines);
            for i = 1:nlines
                xplot(:,i) = [lidarx(i); lidarx(nlines+i)];
                yplot(:,i) = [lidary(i); lidary(nlines+i)];
            end
            
            if any(isempty(map.LidarLinesHandle)) || any(~isvalid(map.LidarLinesHandle))
                map.LidarLinesHandle = plot(ax, xplot, yplot, 'g', 'Tag', 'lidar');
            else
                for i = 1:nlines
                    map.LidarLinesHandle(i).XData = xplot(:,i);
                    map.LidarLinesHandle(i).YData = yplot(:,i);
                end
            end
            
            crosshandles = findobj(ax,'Tag','lidarcross');
            if any(~isempty(crosshandles)) || any(~isvalid(crosshandles))
                delete(crosshandles)
            end
            for i = 1:numSensors
                if lidar(i) < maxLidarDist
                    map.LidarCrossHandle = plot(ax,lidarx(i),lidary(i),'rx','Tag','lidarcross');
                end
            end
        end
        
        function plotTrainingZone(map)
            if isempty(map.TrainZoneHandle) || ~isvalid(map.TrainZoneHandle)
                ax = gca(map.Figure);
                x = [map.TrainingZoneXLimits(1) map.TrainingZoneXLimits(2) map.TrainingZoneXLimits(2) map.TrainingZoneXLimits(1)];
                y = [map.TrainingZoneYLimits(1) map.TrainingZoneYLimits(1) map.TrainingZoneYLimits(2) map.TrainingZoneYLimits(2)];
                map.TrainZoneHandle = fill(ax,x,y,'w','FaceAlpha',0.1);
            end
        end
        
        function plotVehicleStatus(map,status)
            ax = gca(map.Figure);
            txt = "Vehicle Status : " + status;
            if isempty(map.VehicleStatusHandle) || ~isvalid(map.VehicleStatusHandle)
                map.VehicleStatusHandle = text(ax, 2, 52, txt);
            else
                map.VehicleStatusHandle.String = txt;
            end
        end
        
        function plotControllerStatus(map,status)
            ax = gca(map.Figure);
            txt = "Controller Mode : " + status;
            if isempty(map.ControllerStatusHandle) || ~isvalid(map.ControllerStatusHandle)
                map.ControllerStatusHandle = text(ax, 2, 50, txt);
            else
                map.ControllerStatusHandle.String = txt;
            end
        end
        
        function plotTime(map,currentTime)
            ax = gca(map.Figure);
            txt = "Elapsed Time : " + currentTime + "s";
            if isempty(map.TimeDisplayHandle) || ~isvalid(map.TimeDisplayHandle)
                map.TimeDisplayHandle = text(ax, 2, 48, txt);
            else
                map.TimeDisplayHandle.String = txt;
            end
        end
        
        function clearVehicle(map)
            delete(map.VehicleBodyHandle)
            delete(map.VehicleAxlesHandle)
            delete(map.VehicleWheelsHandle)
        end
        
        function clearLidar(map)
            ax = gca(map.Figure);
            if ~isempty(ax) || ~isvalid(ax)
                crosshandles = findobj(ax,'Tag','lidarcross');
                delete(crosshandles)
            end
            delete(map.LidarLinesHandle)
        end
        
        function clearCameraFOV(map)
            delete(map.CameraFOVHandle)
        end
        
        function clearTrainZone(map)
            delete(map.TrainZoneHandle);
        end
        
    end
end

function localValidateIdx(idx)
    if any(idx<0) || any(idx>64) || any(~isnumeric(idx)) || any((idx-floor(idx))~=0)
        error('Index values must be integers between 0 and 64.');
    end
end
% function localHideFigure(src,~)
%     src.Visible = 'off';
% end

    