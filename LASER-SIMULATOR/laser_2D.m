function s = laser_2D(pose, sensor, NOISE)
% MULTIBEAMCOMPLEX2D Simulate laser measurements
%
% DESCRIPTION
%   This  function is similar to "sense_using_laser" but it has more
%   angular range "[-2*pi, 2*pi]".
% 
% OUTPUTS
%       s: Vector that contains the laser scanning [Kx1]
%
% INPUTS
%     pose: Pose [x;y;theta] of the location
%     sensor: Sensor structure
%    NOISE: Flag to indicate the insertion of noise
%               1: The sensor measurements are noise(default)
%               0: The sensor is noise-free
%   DEBUG: Flag for debugging

%----------------------------------------------------------------------%
% 1. SET GLOBAL VARIABLES
%----------------------------------------------------------------------%
%   1.1. ANGLES OF THE LASER IN THE LOCAL FRAME
global theta_laser

%   1.2. MAP OF THE ENVIROMENT
global map



%----------------------------------------------------------------------%
% 2. ARGUMENTS' CHECKING
%----------------------------------------------------------------------%
%  2.1. CHECK NECCESARY ARGUMENTS
if(nargin < 2)
    error('laser_2D: Insuficient number of parameters')
end

%  2.2. MAKE SURE THAT THE "sensor" ARGUMENT IS A STRUCT
if(~isstruct(sensor))
    error('laser_2D: The second argument must be a structure')
end

%  2.3. CHECK THE DIMENSIONS OF THE MAP
if(size(map,2) ~= 2)
    error('laser_2D: The map must have 2 columns(check format)')
end

%  2.4. SET DEFAULT VALUES OF THE FLAG OF NOISE
if(nargin < 3)
    NOISE = 1;      % Noise activated
end


%----------------------------------------------------------------------%
% 3. SENSOR INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  3.1. SET FEATURES OF THE SENSOR
K               = sensor.K;                 % Number of beams
max_range       = sensor.max_range;         % Maximum range 
std_local_noise = sensor.std_local_noise;   % Std of the local measurement noise
max_noise       = sensor.max_noise;         % Prob of. maximum measurement noise
short_noise     = sensor.short_noise;       % Prob of. short measurement noise
%   -> Ubicacion of the sensor in the robot's fixed frame (Pg. 169)
xk  = sensor.xk;
yk  = sensor.yk;
thk = sensor.thk;

%  3.2. SCANNING INITIALIZATION
s = max_range*ones(K,1);


%----------------------------------------------------------------------%
% 4. SET THE POSE OF THE LASER IN THE GLOBAL FRAME
%----------------------------------------------------------------------%
%  4.1. GET THE ORIENTATION OF THE ROBOT
theta = pose(3);

%  4.2. SET GLOBAL LASER POSE
pose(1) = pose(1) + xk*cos(theta) - yk*sin(theta);
pose(2) = pose(2) + xk*sin(theta) + yk*cos(theta);
pose(3) = pose(3) + thk;


%-----------------------------------------------------------------------%
% 5. MAIN CODE
%-----------------------------------------------------------------------%
%  5.1. GET LOCATION AND ORIENTATION OF THE LASER
location = pose(1:2); 
theta = pose(3);

%  5.2. MAIN LOOP
for i = 1:K
    
    % A. DEFINE THE GEOMETRY OF BEAM "i"
    %   -> Set local "global" angle   
    angle = pi_to_pi(theta_laser(i) + theta);
    %   -> Set "maximum" end point
    Tr(1) = location(1) + cos(angle)*max_range;
    Tr(2) = location(2) + sin(angle)*max_range;
    
    
    % B. FIND THE INTERSECTION BETWEEN THE BEAM AND THE WORLD LINES
    [xout, yout] = polyxpoly(map(:,1), map(:,2),...
        [location(1) Tr(1)],[location(2) Tr(2)]);
    %   -> Continue if we have skipped readings
    if(isempty(xout))
        continue;
    end
    
    
    % C. GET THE MINIMUM DISTANCE OF INTERSECTIONS
    if(size(xout,1) > 1)
        mind = inf;
        for k = 1:size(xout,1)
            % Get distance to intersection points
            %d = ptsDistance([xout(k) yout(k)], location(1:2));
            d = norm([xout(k)-location(1),  yout(k)-location(2)]);
            if(d < mind)
                cxout = xout(k);
                cyout = yout(k);
                mind = d;
            end
        end
    else
        % Just 1 intersection here
        cxout = xout;
        cyout = yout;
    end   
    
    
    % D. SET RANGE OF THE BEAM
    s(i) = norm([cxout-location(1), cyout-location(2)]);
end



%----------------------------------------------------------------------%
% 6. ADD NOISE IF DESIRED
%----------------------------------------------------------------------%
if(NOISE)
    % 6.1. SIMULATE LOCAL MEASUREMENT NOISE (gaussian noise) 
    ind = s~=max_range;
    s(ind) = s(ind) + std_local_noise*randn(size(s(ind)));    
    
    % 6.2. SIMULATE SHORT READINGS
    for k=1:K
        if(rand() < short_noise)
            s(k) = s(k)*rand();
        end
    end
    
    % 6.3. SIMULATE "max-readings" FAILURES.
    for k=1:K
        if(rand() < max_noise)
            s(k) = max_range;
        end
    end
end


end