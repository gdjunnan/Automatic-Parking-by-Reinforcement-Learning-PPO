classdef Camera < matlab.System & matlab.system.mixin.CustomIcon & matlab.system.mixin.Propagates
% Camera represents a camera parameterized by a depth and field of
% view.

% Copyright 2020 The MathWorks, Inc.

    % Public, tunable properties
    properties
        MaxDepth = 10
        MaxViewAngle = pi/2
    end
    
    properties (Nontunable)
        % MapObject Map object name
        MapObject char
    end

    properties(DiscreteState)
        Status
    end

    properties(Access = private)
        %Map
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            %obj.Map = evalin('base',obj.MapObject);
        end

        function [isfree,targetpose] = stepImpl(obj,pose)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            map = evalin('base',obj.MapObject);
            
            % get spot locations
            loc      = map.SensorLocations;
            occupied = map.OccupiedSpots;
            
            % extract data.
            locx        = loc(:,1);
            locy        = loc(:,2);
            carx        = pose(1);
            cary        = pose(2);
            cartheta    = mod(pose(3), 2*pi);
            phi         = atan2( (locy-cary) , (locx-carx) );  
            phi         = mod(phi, 2*pi); % phi is the angle with sensor lights wrapped to [0,2*pi]
            phimin      = mod(cartheta - 0.5*obj.MaxViewAngle, 2*pi);
            phimax      = phimin + obj.MaxViewAngle;
            
            % calculate valid angles from all spots
            for i = 1:numel(phi)
                if phimax >= 2*pi && phi(i) <= pi/2
                    phi(i) = 2*pi + phi(i);
                end
            end
            validPhiIdx = (phi >= phimin) & (phi <= phimax);
            
            % calculate valid distances from each spot
            dist = sqrt( (locx-carx).^2 + (locy-cary).^2 );
            validDistIdx = dist <= obj.MaxDepth;
            
            spotsToCheck = validPhiIdx & validDistIdx;
            freeSpots = find(spotsToCheck' & ~occupied, map.MaxSpots);
            targetSpot = min(freeSpots);
            
            isfree = double(~isempty(targetSpot));
            spotfound = obj.Status(1);
            if isfree && spotfound==0
                targetpose  = createTargetPose(map,targetSpot);
                obj.Status = [1 reshape(targetpose, 1, 3)];
            end
            
            targetpose = obj.Status(2:end);
            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.Status = [0 0 0 0];
        end

        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds.Status  = obj.Status;
        end

        function validateInputsImpl(obj,pose)
            % Validate inputs to the step method at initialization
            validateattributes(pose,{'double'},{'numel',3});
        end

        function flag = isInputSizeMutableImpl(obj,index)
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function flag = isInputComplexityMutableImpl(obj,index)
            % Return false if input complexity cannot change
            % between calls to the System object
            flag = false;
        end

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 1;
            % if obj.UseOptionalInput
            %     num = 2;
            % end
        end

        function num = getNumOutputsImpl(obj)
            % Define total number of outputs for system with optional
            % outputs
            num = 2;
            % if obj.UseOptionalOutput
            %     num = 2;
            % end
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = "Camera";
            % icon = ["My","System"]; % Example: multi-line text icon
        end

        function name = getInputNamesImpl(obj)
            % Return input port names for System block
            name = 'pose';
        end

        function [name,name2] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name = 'found';
            name2 = 'goal';
        end

        function [out,out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];
            out2 = [1 3];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out,out2] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";
            out2 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out,out2] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;
            out2 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out,out2] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            % Return size, data type, and complexity of discrete-state
            % specified in name
            sz = [1 4];
            dt = "double";
            cp = false;
        end

        
        
    end

    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"),...
           'Title','Camera',...
           'Text', 'This system object models a camera for identifying free parking spots.');
        end
        
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end
    end
end
