function in = autoParkingValetResetFcn(in)
% Reset function for auto parking valet example

% Copyright 2020 The MathWorks, Inc.

    choice = rand;
    if choice <= 0.35
        x = 37;
        y = 16;
        t = deg2rad(-45 + 2*45*rand);
    elseif choice <= 0.70
        x = 58;
        y = 16;
        t = deg2rad(-225 + 2*45*rand);
    else
        zone = randperm(3,1);
        switch zone
            case 1
                x = 36.5 + (45.5-36.5)*rand;
                t = deg2rad(-45 + 2*45*rand);
            case 2
                x = 45.5 + (50-45.5)*rand;
                t = deg2rad(-135 + 2*45*rand);
            case 3
                x = 50 + (59-50)*rand;
                t = deg2rad(-225 + 2*45*rand);
                if t <= -pi
                    t = 2*pi + t;
                end
        end
        y = 10 + (20-10)*rand;
    end
    
    pose = [x,y,t];
    speed = 5 * rand;
    in = setVariable(in,'egoInitialPose',pose);
    in = setVariable(in,'egoInitialSpeed',speed);
end
