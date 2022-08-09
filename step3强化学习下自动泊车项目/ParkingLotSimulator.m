classdef ParkingLotSimulator < matlab.System & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
% ParkingLotSimulator Plots figure of Parking Lot Environment within a
% Simulink model
%

% Copyright 2020 The MathWorks, Inc.    

    % Public, tunable properties
    properties
        Visualization (1,1) double = 1
    end

    % Public, non-tunable properties
    properties(Nontunable)
        MapObject char
        CameraDepth      (1,1) double
        CameraViewAngle  (1,1) double
        MaxLidarDistance (1,1) double
        TrainingMode     (1,1) double = 0
    end

    properties(DiscreteState)
    end

    % Pre-computed constants
    properties(Access = private)
        %Map
    end

    methods
        % Constructor
        function obj = ParkingLotSimulator(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            %obj.Map = evalin('base',obj.MapObject);
        end

        function stepImpl(obj,pose,steer,lidar,park,currentTime)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            map = evalin('base',obj.MapObject);
            
            if obj.Visualization > 0
                % Plot vehicle
                plotVehicle(map,pose,steer);

                % Plot/clear training zone
                if obj.TrainingMode
                    plotTrainingZone(map);
                else
                    clearTrainZone(map);
                end

                % Plot/clear lidar and camera FOV
                if obj.TrainingMode || park
                    clearCameraFOV(map);
                    plotLidar(map,pose,lidar,obj.MaxLidarDistance);
                    if obj.TrainingMode
                        plotVehicleStatus(map,'TRAIN');
                    else
                        plotVehicleStatus(map,'PARK');
                    end
                    plotControllerStatus(map,'RL');
                else
                    clearLidar(map);
                    plotCameraFOV(map,pose,obj.CameraDepth,obj.CameraViewAngle);
                    plotVehicleStatus(map,'SEARCH');
                    plotControllerStatus(map,'MPC');
                end
                
                if round(currentTime,1) > floor(currentTime)
                    plotTime(map,round(currentTime,1));
                end
                
                drawnow %limitrate
            end
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            %map = evalin('base',obj.MapObject);
            %clearVehicle(map);
            %clearLidar(map);
            %clearCameraFOV(map);
            %show(map);
        end

        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            %s.myproperty = obj.myproperty;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            % obj.myproperty = s.myproperty; 

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        %% Simulink functions
        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end

        function flag = isInputSizeMutableImpl(obj,index)
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [0 0];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = mfilename("class"); % Use class name
            % icon = "My System"; % Example: text icon
            % icon = ["My","System"]; % Example: multi-line text icon
            % icon = matlab.system.display.Icon("myicon.jpg"); % Example: image file icon
        end
    end

    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"),...
           'Title','Parking Lot Simulator',...
           'Text', 'This system object simulates a Parking Lot with obstacles.');
        end

        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename("class"));
        end

        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
end
