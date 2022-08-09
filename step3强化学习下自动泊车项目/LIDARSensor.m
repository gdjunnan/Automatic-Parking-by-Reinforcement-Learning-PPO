classdef LIDARSensor < matlab.System & ...
        matlab.system.mixin.Propagates
% Lidar sensor model

% Copyright 2020 The MathWorks, Inc.

    properties
        % Rectangles global position and orientation specified as a N x 5
        % vector [x,y,L,W,theta]
        %Rectangles = [5,5,1,3,0;5,-5,1,3,0]
    end
    properties (Nontunable)
        % MapObject Name of ParkingLot object
        MapObject = ''
        
        % Geometry of the car [L,W]
        CarGeometry = [3,1]
        
        % Max distance reading of the sensor
        MaxDistance = 12
    end
    properties (Nontunable,PositiveInteger)
        % Number of sensor readings available
        SensorResolution = 60; %numObsLidar
    end
    properties
        % Show the sensor plot
        ShowLIDARPlot = 0
    end
    properties
        % Limits of the plot [xLower,xUpper,yLower,yUpper]
        PlotLimits = [-20,20,-20,20]
    end
    properties (Access = private)
        Rectangles
    end
    methods 
        function this = LIDARSensor(varargin)
            setProperties(this,nargin,varargin{:});
        end
    end
    methods (Access = protected)
        function d = outputImpl(this,botx,boty,bottheta)
            
            % get data from props
            sensorRes = this.SensorResolution;
            maxDistance = this.MaxDistance; % meters
            
            % create the segs from the rectangles
            %rect = this.Rectangles(~all(this.Rectangles==zeros(1,5),2),:);
            map = evalin('base', this.MapObject);
            rect = map.ObstacleMatrix;
            [segsx,segsy] = rect2segs(rect);
            
            % xform the segments given the bot position
            s = sin(bottheta); c = cos(bottheta);
            R = [c,-s;s,c];
            
            x_ = segsx(:)';
            y_ = segsy(:)';
            
            z = (R')*([x_;y_] - [botx;boty]);
            
            segsx_xform = reshape(z(1,:)',size(segsx));
            segsy_xform = reshape(z(2,:)',size(segsy));
            
            % generate the lidar data
            [d,intersections,~,lidarx,lidary] = lidarSegmentIntersections(...
                sensorRes,maxDistance,segsx_xform,segsy_xform);
            
            if this.ShowLIDARPlot
                plotLIDAR(this,botx,boty,bottheta,R,segsx,segsy,lidarx,lidary,intersections);
            end
        end
        function updateImpl(this,x,y,theta) %#ok<INUSD>
        end
        function setupImpl(this)
            map = evalin('base', this.MapObject);
            this.Rectangles = map.ObstacleMatrix;
        end
        function resetImpl(this) %#ok<MANU>
        end
        function plotLIDAR(this,botx,boty,bottheta,R,segsx,segsy,lidarx,lidary,intersections)
            persistent f
            if isempty(f) || ~isvalid(f)
%                 f = figure(...
%                     'Toolbar','none',...
%                     'NumberTitle','off',...
%                     'Name','LIDAR Sensor',...
%                     'MenuBar','none');
                f = figure('NumberTitle','off',...
                    'Name','LIDAR Sensor');
                ha = gca(f);
                grid(ha,'on');
            end
            
            ha = gca(f);
            
            ha.XLim = this.PlotLimits(1:2);
            ha.YLim = this.PlotLimits(3:4);
            
            % get the car distances for plotting
            res = size(lidarx,1);
            angles = ((0:(res-1))*2*pi/res)';
            cd = getCarSegmentLengths(this.CarGeometry(1),this.CarGeometry(2),angles);
            
            cdx = cd.*cos(angles);
            cdy = cd.*sin(angles);
            
            lidarx(:,1) = cdx(:);
            lidary(:,1) = cdy(:);
            
            % xform the lidar data for plotting
            x_ = lidarx(:)';
            y_ = lidary(:)';
            
            z = R*[x_;y_] + [botx;boty];
            
            lidarx_xform = reshape(z(1,:)',size(lidarx));
            lidary_xform = reshape(z(2,:)',size(lidary));
            
            ix = [];
            for i = 1:numel(intersections)
                intersection = intersections{i};
                if ~isempty(intersection)
                    intersection = (R*intersection' + [botx;boty])';
                    ix(end+1,:) = intersection; %#ok<AGROW>
                end
            end
            
            % plot
            lh = findobj(ha,'Tag','seg_lh');
            if isempty(lh) || numel(lh) ~= size(segsx,1)
                delete(lh);
                line(ha,segsx',segsy','Color','k','Tag','seg_lh');
            else
                for i = 1:numel(lh)
                    set(lh(i),'XData',segsx(i,:)','YData',segsy(i,:)');
                end
            end

            lh = findobj(ha,'Tag','lidar_lh');
            if isempty(lh) || numel(lh) ~= this.SensorResolution
                delete(lh);
                % lidar objs
                lh = line(ha,lidarx_xform',lidary_xform','Color','g','Tag','lidar_lh');
                % the first line indicates orientation
                lh(1).Color = 'k';
            else
                % flip to preserve graphics order
                lh = flip(lh);
                for i = 1:this.SensorResolution
                    set(lh(i),'XData',lidarx_xform(i,:)','YData',lidary_xform(i,:)');
                end
            end
            
            intersection_lh = findobj(ha,'Tag','intersect_lh');
            delete(intersection_lh);
            if ~isempty(ix)
                line(ha,ix(:,1),ix(:,2),'Color','r','Marker','x','LineStyle','none','Tag','intersect_lh'); 
            end
            
            % plot the car
            [carsegsx,carsegsy] = rect2segs([botx,boty,this.CarGeometry,bottheta]);
            lh = findobj(ha,'Tag','car_lh');
            if isempty(lh) || numel(lh) ~= size(carsegsx,1)
                delete(lh);
                line(ha,carsegsx',carsegsy','Color','k','LineStyle','-','Tag','car_lh');
            else
                for i = 1:numel(lh)
                    set(lh(i),'XData',carsegsx(i,:)','YData',carsegsy(i,:)');
                end
            end
            
        end
        function releaseImpl(~)
        end

        function num = getNumInputsImpl(~)
            % Define total number of inputs for system with optional inputs
            num = 3;
        end
        function num = getNumOutputsImpl(~)
            % Define total number of outputs for system with optional
            % outputs
            num = 1;
        end
        function loadObjectImpl(this,s,wasLocked)
            % Needed for fast restart
            
            % Set public properties and states
            loadObjectImpl@matlab.System(this,s,wasLocked);
        end

        function s = saveObjectImpl(this)
            % Needed for fast restart
            
            % Set public properties and states
            s = saveObjectImpl@matlab.System(this);
        end

        function varargout = getInputNamesImpl(~)
            % Return input port names for System block
            varargout{1} = 'botx';
            varargout{2} = 'boty';
            varargout{3} = 'theta';
        end

        function varargout = getOutputNamesImpl(~)
            % Return output port names for System block
            varargout{1} = 'd';
        end
        function varargout = isInputDirectFeedthroughImpl(this,varargin)
            % when TreatAsDirectFeedthrough is false, implicit unit delays
            % are added to the input signals. In most cases
            % TreatAsDirectFeedthrough should be true
            
            nu = nargin(this);
            varargout = cell(1,nu);
            for i = 1:nu
                varargout{i} = true;
            end
        end
        function varargout = getOutputSizeImpl(this)
            % get the size and sample time from the agent
            varargout{1} = [this.SensorResolution,1];
        end
        function varargout = isOutputComplexImpl(this)
            ny = nargout(this);
            varargout = cell(1,ny);
            for i = 1:ny
                varargout{i} = false;
            end
        end
        function varargout = getOutputDataTypeImpl(~)
            varargout{1} = 'double';
        end
        function varargout = isOutputFixedSizeImpl(this)
            ny = nargout(this);
            varargout = cell(1,ny);
            for i = 1:ny
                varargout{i} = true;
            end
        end
    end
    methods(Access = protected,Static)
        function simMode = getSimulateUsingImpl
            % interpreted execution is only allowed
            simMode = "Interpreted execution";
        end
    end
end