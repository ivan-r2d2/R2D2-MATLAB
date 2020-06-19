% LASER Laser sensor range finder
%
%   This class simulates a laser range finder (Hoyuko, SICK) used in 
%   mobile robots for tasks such as localication and mapping
classdef laser < handle
    properties (SetAccess=private, GetAccess=public)
        % MAP OF THE ENVIRONMENT(Made of lines)
        map                 % Format used by "sense_using_laser"
        map2                % Format used by "laser_2D"
        IS_READY=0          % FLAG (1:Sensor is ready / 0:No)
        
        % MODE OF OPERATION
        MODE                % NORMAL: [-pi/2, pi/2] (default)
                            %   FULL: [-120, 120]
                            %   FREE: Angles set by the user
        
        % ATTRIBUTES OF THE SENSOR
        K                   % Number of beams
        max_range           % Maximum range 
        std_local_noise     % Std of. local measurement noise
        max_noise;          % Probability of failures (max. measurement)
        short_noise;        % Probability of unexpected objects
        
        % POSE OF THE SENSOR IN THE ROBOT'S FIXED FRAME (Pg. 169)
        xk                  % Axis in the direction of motion
        yk 
        thk                 % Orientation with respect to axis "xk"
        
        % ANGLES OF THE LASER IN THE LASER'S LOCAL FRAME[-pi/2, pi/2]
        theta_laser
        sin_theta_laser  
        cos_theta_laser
        
        % ANGLES OF THE LASER USED INTERNALLY BY SOME METHODS[0, pi]
        theta_laser_sense  
        sin_theta_laser_sense  
        cos_theta_laser_sense
        
        % ANGLES OF THE BEAMS SELECTED BY THE USER
        theta_laser_SELECTED_INDEX      % [K_SELECTEDx1]
        theta_laser_SELECTED
        K_SELECTED                      % Number of selected rays
        USE_SELECTED_ANGLES = 0         % FLAG (1: Use selected angles)
        
        % TO VISUALIZE THE END POINTS OF THE LASER
        h_figure            % Handle of the figure
        h_points            % Handles to plot the laser's end point
    end
    
    
    %------------------------------------------------------------------%
    %   PUBLIC METHODS
    %------------------------------------------------------------------%
    methods
        %---------------------- CONSTRUCTOR --------------------------%
        function laser = laser(varargin)
            % laser.laser  Constructor function
            %
            %   LASER = LASER() returns a valid "laser" object. 
            %
            %   LASER = LASER(MAP) returns a valid "laser" object. Also
            %           initialize the properties map and map2
            
            %----------------------------------------------------------%
            % 1. SET THE INITIAL MAP IF DESIRED
            %----------------------------------------------------------%
            if(nargin >= 1)
                %  1.1. CHECK THE FIRST ARGUMENT
                map_str = varargin{1};
                if(~ischar(map_str))
                    error('LASER.LASER: Argument must be a string')
                end
                %  1.2. LOAD THE MAP
                load(map_str, 'map') 
                %  1.3. SET THE MAP
                laser.map = map;
                %  1.4. GET THE MAP IN THE OTHER FORMAT (map2)
                laser.transform_map();          
            end
            
            %-----------------------------------------------------------%
            % 2. SET MODE OF OPERATION
            %-----------------------------------------------------------%
            laser.MODE = 'NORMAL';      % By default
        end
        
        function set_mode_operation(this, mode, K)
            % laser.set_mode_operation Set mode of operation
            %
            %   SET_MODE_OPERATION(MODE) Set mode of operation of the laser
            %   Especifically we set the function to simulate the laser.
            %
            %       MODE = 'FULL'   -> Use 'sense_using_laser_full'
            %       MODE = 'NORMAL' -> Use 'sense_using_laser_normal'
            %       MODE = 'FREE'   -> Use 'sense_using_laser_full' but the
            %                          user set the angles
            
            %----------------------------------------------------------%
            % 1. SET THE ANGLES
            %----------------------------------------------------------%
            switch(mode)
                case 'NORMAL'
                    % SET ANGLES OF THE LASER
                    theta_min = -pi/2;      % Set minimum angle
                    theta_max =  pi/2;      % Set maximum angle
                    theta_LASER = linspace(theta_min, theta_max, K);
                    % CALL METHOD TO SET THE ANGLES
                    this.set_theta_laser(theta_LASER);
                case 'FULL'
                    % SET ANGLES OF THE LASER
                    theta_min = -120*pi/180;    % Set minimum angle
                    theta_max =  120*pi/180;    % Set maximum angle
                    theta_LASER = linspace(theta_min, theta_max, K);
                    % CALL METHOD TO SET THE ANGLES
                    this.set_theta_laser(theta_LASER);
                case 'FREE'
                    % LET THE USER SET THE ANGLES
                otherwise
                    str = ['laser.set_mode_operation:'...
                        'Mode of operation not defined'];
                    error(str);
            end
            
            %----------------------------------------------------------%
            % 2. SET MODE OF OPERATION
            %----------------------------------------------------------%
            this.MODE = mode;
        end
        
        function use_laser_hokuyo_URG_04LX_UG01(this)
            % laser.use_laser_Hokuyo_URG_04LX_UG01 Sets the laser as the
            %                                      Kokuyo URG-04LX-UG01
            %
            %   Call this method if you are going to use the Hokuyo 
            %   URG-04LX-UG01 range finder
            
            %-----------------------------------------------------------%
            % 1. SET INTERNAL ATTRIBUTES OF THE Kokuyo URG-04LX-UG01
            %-----------------------------------------------------------%
            this.max_range       = 5.6;
            this.std_local_noise = 0.01;
            this.short_noise     = 0.05;
            this.max_noise       = 0.02;
        end
        
        function add_line(this, pA, pB)
            % laser.add_line Add line to the map
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. IF THE USER DOES NOT PASS THE POINTS
            if(nargin<2)
                pA = ginput(1);
                pB = ginput(1);
            end
            
            %-----------------------------------------------------------%
            % 2. ADD THE LINE TO THE MAP
            %-----------------------------------------------------------%
            MAP = this.map;
            MAP(end+1,:) = [pA pB];
            
            %-----------------------------------------------------------%
            % 3. UPDATE MAPS
            %-----------------------------------------------------------%
            this.map = MAP;
            this.transform_map();    
        end
        
        function add_lines(this,N)
            % laser.add_lines Add lines to the map using the mouse
            for i=1:N
                this.add_line();
            end
        end
        %-------------------- END CONSTRUCTOR ------------------------%
        
        
        %--------------- BASIC LASER FUNCTIONS -----------------------%
        function plot_map(this, color)
            % laser.plot_map Plot the map given to the laser
            %
            %   PLOT_MAP plot the map(composed of lines) of the laser
            %            using the current axes.
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK IF THE MAP HAS BEEN DEFINED
            if(isempty(this.map))
                str = ['laser.plot_map'...
                    'The laser does not a have a line map'];
                error(str);
            end
            %  1.2. SET DEFAULT COLOR
            if(nargin<2)
                color = 'r';
            end
            
            %-----------------------------------------------------------%
            % 2. PLOT THE MAP 
            %-----------------------------------------------------------%
            %  2.1. GET MAP AND SET ADDITIONAL OFFSET OF THE AXES
            MAP = this.map;
            axes_offset = 0.5;
            %  2.2. COMPUTE THE MAX AND MIN VALUES OF THE COORDINATES OF
            %       THE LINES
            %   -> Along the x-axes
            xmin_map = min(min(MAP(:,1), MAP(:,3)));
            xmax_map = max(max(MAP(:,1), MAP(:,3)));
            %   -> Along the y-axes
            ymin_map = min(min(MAP(:,2), MAP(:,4)));
            ymax_map = max(max(MAP(:,2), MAP(:,4)));
            %  2.3. DRAW THE MAP IN THE CURRENT AXES
            %   -> Draw using lines
            line([MAP(:,1) MAP(:,3)]', [MAP(:,2) MAP(:,4)]','color',color)
            %   -> Set correct axis
            axis([xmin_map-axes_offset   xmax_map+axes_offset...
                    ymin_map-axes_offset   ymax_map+axes_offset])
        end   
        
        function set_theta_laser(laser, theta_laser)
            % laser.set_theta_laser Set the angles of the beams
            %
            %   SET_THETA_LASER(THETA_LASER) Set the angles of the beams
            %   using the local frame of the robot
            %
            % INPUTS
            %   theta_laser: The vector that defines the angles.
            %
            % NOTE:
            %   - If the user calls this method the laser is put in mode
            %     'FREE'. Otherwise the laser stays in its defined mode.
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['laser.set_theta_laser'...
                    'Insufficient number of arguments'];
                error(str);
            end
            
            %-----------------------------------------------------------%
            % 2. SET PROPERTIES
            %-----------------------------------------------------------%
            %  2.1. SET THE NUMBER OF BEAMS
            laser.K = length(theta_laser);
            %  2.2. SET ANGLES OF THE LASER
            laser.theta_laser     = theta_laser(:);
            laser.sin_theta_laser = sin(laser.theta_laser);
            laser.cos_theta_laser = cos(laser.theta_laser);
            %  2.3. SET "internal" ANGLES OF THE LASER
            laser.theta_laser_sense     = laser.theta_laser + pi/2; 
            laser.sin_theta_laser_sense = sin(laser.theta_laser_sense);
            laser.cos_theta_laser_sense = cos(laser.theta_laser_sense);
            %  2.4. SET MODE OF THE LASER AS 'FREE'
            laser.MODE = 'FREE';
        end  
        
        function theta_laser_selected = ...
                set_theta_selected_index(laser, theta_laser_selected_index)
            % laser.set_theta_laser_selected_index Set the selected angles
            %
            %   SET_THETA_SELECTED_INDEX Set the index of the selected 
            %   angles to be used by the user
            %
            % INPUTS:
            %   theta_laser_selected_index: Indexes of the selected angles. 
            %                               If this is a number it represents 
            %                               the number of beams
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. CHECK "theta_laser"
            if(isempty(laser.theta_laser))
                str = ['laser.set_theta_laser_selected_index:'...
                    'theta_laser is not defined'];
                error(str)
            end
            %  1.2. SET "theta_laser_selected_index" IF THE INPUT IS A NUMBER 
            if(length(theta_laser_selected_index) == 1)
                theta_laser_selected_index = ceil(linspace(1, laser.K, ...
                    theta_laser_selected_index));
            end
            
            %-----------------------------------------------------------%
            % 2. SET THE SELECTED ANGLES
            %-----------------------------------------------------------%
            %  2.1 SET THE INDEXES
            laser.theta_laser_SELECTED_INDEX = theta_laser_selected_index(:);           
            %  2.2 SET THE SELECTED ANGLES
            theta_laser_selected       = laser.theta_laser(...
                theta_laser_selected_index); 
            laser.theta_laser_SELECTED = theta_laser_selected; 
            %  2.3 SET FLAG
            laser.USE_SELECTED_ANGLES = 1;
            laser.K_SELECTED = length(theta_laser_selected);
        end
        
        function set_max_range(laser, max_range)
            % laser.set_max_range Set maximum range range of the laser
            %
            %   SET_MAX_RANGE(MAX_RANGE) Set the maximum range of the laser
            %   range finder
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['laser.set_max_range'...
                    'Insufficient number of arguments'];
                error(str);
            end
            
            %-----------------------------------------------------------%
            % 2. SET THE PROPERTY
            %-----------------------------------------------------------%
            laser.max_range = max_range;
        end
            
        function set_std_local_noise(laser, std_noise)
            % laser.set_std_local_noise Set std of the local gaussian noise
            %
            %   SET_STD_LOCAL_NOISE(NOISE) Set std of the local gaussian 
            %   noise of the measurements
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['laser.set_std_local_noise'...
                    'Insufficient number of arguments'];
                error(str);
            end
            
            %-----------------------------------------------------------%
            % 2. SET THE PROPERTY
            %-----------------------------------------------------------%
            laser.std_local_noise = std_noise;
        end
        
        function set_short_noise(laser, p_short)
            % laser.set_short_noise Set probability of short measurements
            %
            %   SET_SHORT_NOISE(NOISE) Set the probability of short
            %   measurements due to unexpected objects.
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['laser.set_short_noise'...
                    'Insufficient number of arguments'];
                error(str);
            end
            
            %-----------------------------------------------------------%
            % 2. SET THE PROPERTY
            %-----------------------------------------------------------%
            laser.short_noise = p_short;
        end
        
        function set_max_noise(laser, p_max)
            % laser.set_max_noise Set probability of max measurements
            %
            %   SET_MAX_NOISE(NOISE) Set the probability of maximum
            %   measurements.
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['laser.set_max_noise'...
                    'Insufficient number of arguments'];
                error(str);
            end
            
            %-----------------------------------------------------------%
            % 2. SET THE PROPERTY
            %-----------------------------------------------------------%
            laser.max_noise = p_max;
        end
        
        function set_local_pose(laser, local_pose)
            % laser.set_local_pose Set the laser local pose
            %
            %   SET_LOCAL_POSE Set the pose of the sensor in the 
            %   robot's fixed frame
            %
            % INPUT
            %   local_pose: Pose of the laser in the robot's local frame          
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK DIMENSION OF THE LOCAL POSE
            if(length(local_pose)~=3)
                error('laser.set_local_pose: Local pose must be of size 3')
            end
            %  1.2. CHECK THE LOCAL ORIENTATION
            local_pose(3) = pi_to_pi(local_pose(3));
            
            %-----------------------------------------------------------%
            % 2. SET LOCAL POSE
            %-----------------------------------------------------------%
            laser.xk  = local_pose(1);
            laser.yk  = local_pose(2);
            laser.thk = local_pose(3);
        end
        
        function transform_map(laser)
            % laser.transform_map Convert our map format to a new format
            %
            %   TRANSFORM_MAP(MAP) Transform our original map to a new
            %   format in order to be used by the function "laser_2D"

            %----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %----------------------------------------------------------%
            %  1.1. CHECK THE EXISTENCE OF THE MAP
            if(isempty(laser.map))
               error('laser.TRANSFORM_MAP: The map is not defined') 
            end
            %  1.2. NUMBER OF LINES
            n_rows = size(laser.map,1);   
            
            
            %----------------------------------------------------------%
            % 2. COMPUTE THE NEW MAP REPRESENTACION
            %----------------------------------------------------------%
            %  2.1. ALLOCATE SPACE
            new_map = zeros(3*n_rows,2);
            %  2.2. MAIN CODE
            for i=1:n_rows
                ind = 3*(i-1) + 1;
                new_map(ind,:)   = laser.map(i,1:2);
                new_map(ind+1,:) = laser.map(i,3:4);
                new_map(ind+2,:) = [NaN NaN];
            end
            %  2.3. SET PROPERTY
            laser.map2 = new_map;
        end
        %--------------- END BASIC LASER FUNCTIONS -------------------%
        
        
        %------------------ MAIN LASER FUNCTIONS ---------------------%
        function [s_full, s] = sense_using_laser(this, pose, NOISE, DEBUG)
            % robot.sense_using_laser  Simulate the measurements of a 
            %                          laser range finder
            %
            %   SENSE_USING_LASER_NORMAL Simulates the measurements of the
            %   laser in the pose "POSE" using the "map" given to the object.
            %
            % OUTPUT
            %   s_full: Vector that contains the full laser scanning [Kx1]
            %        s: Vector that contains the selected beams. If the
            %           user has not made any selection, s is empty "[]"
            % INPUTS
            %   pose: Pose of the robot
            %  NOISE: Flag to indicate the insertion of noise
            %           1: The sensor measurements are noise(default)
            %           0: The sensor is noise-free
            %  DEBUG: Flag for debugging
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['laser.sense_using_laser'...
                    'Insufficient number of arguments'];
                error(str);
            end
            %  1.2. SET DEFAULT VALUE OF THE NOISE FLAG
            if(nargin < 3)
                NOISE = 1;
            end
            %  1.3. SET DEFAULT VALUE OF THE DEBUG FLAG
            if(nargin < 4)
                DEBUG = 0;
            end
                        
            %-----------------------------------------------------------%
            % 2. CHECK READINESS
            %-----------------------------------------------------------%
            if(this.IS_READY==1)
                this.check_readiness(NOISE);
            end
                        
            %-----------------------------------------------------------%
            % 3. SET THE POSE OF THE LASER IN THE GLOBAL FRAME
            %-----------------------------------------------------------%
            %  3.1. GET LOCAL POSE OF THE LASER
            xxk  = this.xk;
            yyk  = this.yk;
            tthk = this.thk;
            %  3.2. GET THE ORIENTATION OF THE ROBOT
            theta = pose(3);
            %  3.3. SET GLOBAL LASER POSE
            pose(1) = pose(1) + xxk*cos(theta) - yyk*sin(theta);
            pose(2) = pose(2) + xxk*sin(theta) + yyk*cos(theta);
            pose(3) = pose(3) + tthk;
            
            %-----------------------------------------------------------%
            % 4. CALL THE APPROPIATE METHODS
            %-----------------------------------------------------------%
            switch(this.MODE)
                case 'NORMAL'   % 'sense_using_laser_normal'
                    s_full = this.sense_using_laser_normal(pose, NOISE, DEBUG);
                    
                case {'FULL','FREE'}   % 'sense_using_laser_full'
                    s_full = this.sense_using_laser_full(pose, NOISE);
            end
            
            %-----------------------------------------------------------%
            % 5. RETURN THE SELECTED BEAMS
            %-----------------------------------------------------------%
            s = [];
            if this.USE_SELECTED_ANGLES==1
                s = s_full(this.theta_laser_SELECTED_INDEX);
            end
        end
        
        function check_readiness(this, NOISE)
            % laser.check_readiness
            %
            %   CHECK_READINESS Check if the laser is ready to simulate
            %   measurements
            
            %-----------------------------------------------------------%
            % 1. CHECK NECCESARY PROPERTIES
            %-----------------------------------------------------------%
            %  1.1. CHECK EXISTENCE OF THE MAP
            if(isempty(this.map))
                error('laser.check_readiness: The laser does not have a map');
            end
            %  1.2. CHECK THE "max_range"
            if(isempty(this.max_range))
                error('laser.check_readiness: max_range is not defined')
            end
            %  1.3. CHECK NOISE PARAMETERS
            if(NOISE)
                if(isempty(this.std_local_noise))
                    error('laser.check_readiness: std_local_noise is not defined')
                end
                if(isempty(this.max_noise))
                    error('laser.check_readiness: max_noise is not defined')
                end
                if(isempty(this.short_noise))
                    error('laser.check_readiness: short_noise is not defined')
                end
            end
            
            %-----------------------------------------------------------%
            % 2. SET FLAG
            %-----------------------------------------------------------%
            this.IS_READY = 1;
        end
        %---------------- END MAIN LASER FUNCTIONS -------------------%
        
        
        %----------------  VISUALIZATION FUNCTIONS -------------------%
        function display_laser(this, pose, s, handle_laser)
            % laser.display_laser Display the beams of the laser
            %
            %   DISPLAY_LASER Plot the laser beams found by the range
            %   finder using the specified handlers
            %
            % INPUTS
            %         pose: The pose of the robot
            %            s: Vector that contains the laser scanning [Kx1]
            % handle_laser: Handles to draw the lines.
            %
            %  NOTE: 's' must be 's_full' o 's' (See sense_using_laser)
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 4)
                str = ['laser.display_laser:',...
                    'Insuficient number of parameters'];
                error(str);
            end
            %  1.2. CHECK THE ANGLES OF THE BEAMS
            if(isempty(this.theta_laser))
                str = ['laser.display_laser:',...
                    'Angles of the beams not defined'];
                error(str) 
            end
            %  1.3. SET BEAMS TO BE PLOTTED
            if(length(s) == this.K)  % Full laser
                THETA_LASER = this.theta_laser;
                KK = this.K;
            elseif(length(s) == this.K_SELECTED) % Selected beams
                THETA_LASER = this.theta_laser_SELECTED;
                KK = length(THETA_LASER);
            else
                str = ['laser.display_laser:',...
                    'S has not the size of K or K_SELECTED'];
                error(str)
            end
            %  1.4. CHECK HANDLE DIMENSION
            if(length(handle_laser) ~= KK+1)
                str = ['laser.display_laser:',...
                    'handle_laser must be of dimension K+1'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1 GET THE LOCATION AND ORIENTATION OF THE ROBOT
            xx = pose(1);
            yy = pose(2);
            theta = pose(3);
            %  2.2. POSE OF THE SENSOR IN THE ROBOT'S FIXED FRAME (Pg. 169)
            xxk  = this.xk;
            yyk  = this.yk;
            tthk = this.thk;
            
            %-----------------------------------------------------------%
            % 3. MAIN CODE - PLOT THE LINES
            %-----------------------------------------------------------%
            %  3.1. COORDINATES OF THE BEAM RAY "on the robot".
            x1 = (xx + xxk*cos(theta) - yyk*sin(theta))*ones(1,KK);
            y1 = (yy + xxk*sin(theta) + yyk*cos(theta))*ones(1,KK);
            %  3.2. COORDINATES OF THE END OF THE BEAM(INTERS. POINT) 
            x2 = zeros(1,KK);
            y2 = zeros(1,KK);
            for k=1:KK
                % Get the coordinates of the contact point of the beam
                x2(k) = xx + xxk*cos(theta) - yyk*sin(theta) +...
                    s(k)*cos(theta + THETA_LASER(k) + tthk);
                y2(k) = yy + xxk*sin(theta) + yyk*cos(theta) + ...
                    s(k)*sin(theta + THETA_LASER(k) + tthk);
                % Plot the lines using the handlers
                set(handle_laser(1,k),'xdata',[x1(k), x2(k)],...
                    'ydata',[y1(k), y2(k)]);
            end
            %  3.3. PLOT THE UBICATION OF THE LASER
            set(handle_laser(1,KK+1), 'xdata', x1, 'ydata', y1);
        end
        
        function plot_laser_end_points(laser, s)
            % laser.plot_laser_end_points Plot the laser "end-points" viewed 
            %                             from the laser
            %
            %   PLOT_LASER_END_POINTS(S) Function to show the laser
            %   "end-points" from the point of the view of the laser
            %
            % INPUT
            %   s: Vector that contain the "full" laser scans    
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['laser.PLOT_LASER_END_POINTS:',...
                    'Insuficient number of parameters'];
                error(str)
            end
            %  1.2. CHECK FOR THE ANGLES OF THE BEAMS
            if(isempty(laser.theta_laser_sense))
                str = ['laser.PLOT_LASER_END_POINTS:',...
                    'Angles of the beams not defined'];
                error(str) 
            end
            %  1.3. CHECK THE FIGURE TO DISPLAY THE END POINTS
            if(isempty(laser.h_figure))
                % Call the method to create the figure
                laser.create_laser_figure();
            end
            %  1.4. CHECK DIMENSION OF "s"
            if(length(s)~=laser.K)
                str = ['laser.PLOT_LASER_END_POINTS:',...
                    'Dimension of s must be equal to K'];
                error(str) 
            end
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1. HANDLE TO PREVIOUS FIGURE
            h_previous_fig = gcf;
            %  2.2. GET LASER FIGURE
            figure(laser.h_figure);
            %  2.3. GET SOME PROPERTIES
            THETA_LASER_SENSE = laser.theta_laser_sense;
            MAX_RANGE         = laser.max_range;
            
            %-----------------------------------------------------------%
            % 3. PLOT OF THE POINTS OF CONTACT
            %   -> Notice that since we consider the measures from the 
            %      view of the robot. 
            %-----------------------------------------------------------%
            %  3.1. PLOT THE LASER MEASUREMENTS.
            %   -> Convert from "polar" to "cartesian" coordinates
            [s_x, s_y] = pol2cart(THETA_LASER_SENSE, s);
            %   -> Do the plotting
            plot(s_x,s_y,'*b');
            %  3.2. PLOT THE MAX RANGE MEASUREMENTS
            [s_x,s_y] = pol2cart(THETA_LASER_SENSE, ((s)==MAX_RANGE).*s);
            plot(s_x,s_y,'*k');
            %  3.3. PLOT OTHER THINGS
            %   -> Plot the origin as a blue point
            plot(0,0,'*b','MarkerSize',10);
            %   -> Set other attributes
            axis([-1.0  1.0  -0.5  1.0]*1.1*MAX_RANGE);
            %  3.4. SET PREVIOUS FIGURE
            figure(h_previous_fig);
        end
        
        function display_laser_end_points(laser, s)
            % robot.display_laser_end_points Plot the laser "end-points"
            %                                using handles
            %
            %   DISPLAY_LASER_END_POINTS(S) Function to display the laser
            %   end points from point of the view of the laser
            %
            % INPUT
            %   s: Vector that contains laser scans    
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['laser.display_laser_end_points:',...
                    'Insuficient number of parameters'];
                error(str)
            end
            %  1.2. CHECK FOR THE ANGLES OF THE BEAMS
            if isempty(laser.theta_laser)
                str = ['laser.display_laser_end_points:',...
                    'Angles of the beams not defined'];
                error(str) 
            end
            %  1.3. CHECK THE FIGURE TO DISPLAY THE END POINTS
            if isempty(laser.h_figure)
                laser.create_laser_figure();
            end
            
            %-----------------------------------------------------------%
            % 2. SET THE INDEXES AND THE ANGLES OF THE BEAMS 
            %-----------------------------------------------------------%
            if(length(s) == laser.K)     % PLOT ALL THE RAYS
                THETA_LASER = laser.theta_laser_sense;
            else % PLOT THE SELECTED RAYS
                if(isempty(laser.theta_laser_SELECTED_INDEX))
                    str = ['laser.display_laser_end_points:',...
                        'Index of the selected angles are not defined'];
                    error(str) 
                end
                THETA_LASER = laser.theta_laser_SELECTED + pi/2;
            end
            
            %-----------------------------------------------------------%
            % 3. PLOT OF THE POINTS OF CONTACT
            %-----------------------------------------------------------%
            %  3.1. CONVERT THE CARTESIAN COORDINATES
            [points_x, points_y] = pol2cart(THETA_LASER, s);
            %  3.2. PLOT USING THE HANDLERS
            set(laser.h_points, 'xdata', points_x, 'ydata', points_y)
        end
        
        function create_laser_figure(laser)
            % laser.create_laser_figure Creates a figure to plot laser 
            %                           "end-points"
            % 
            %   CREATE_LASER_FIGURE Creates a figure to plot the laser
            %   "end-points" from the lasers'point of view
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 1)
                str = ['laser.create_laser_figure'...
                    'Insufficient number of arguments'];
                error(str);
            end
            %  1.2 CHECK FOR THE MAX RANG
            if(isempty(laser.max_range))
                str = ['laser.create_laser_figure'...
                    'The max-range is not defined'];
                error(str);
            end
            %  1.3. HANDLE TO PREVIOUS FIGURE
            h_previous_fig = gcf;
            
            %-----------------------------------------------------------%
            % 2. CREATE THE FIGURE
            %-----------------------------------------------------------%
            %  2.1. CREATE THE FIGURE AND SAVE THE HANDLE
            laser.h_figure = figure();    
            set(laser.h_figure,'position',[10 440 360 250],...
                'Name','SENSOR MEASUREMENTS',...
                'color',[211 208 200]/255, 'Visible','on');
            %  2.2. ADDITIONAL PROPERTIES
            set(gca, 'Box', 'on')
            hold on
            grid on
            xlabel('x(m)')
            ylabel('y(m)')
            axis([-1.0  1.0  -0.5  1.0]*1.1*laser.max_range)
            %  2.3. SET HANDLES TO PLOT THE LASER END POINTS
            laser.h_points = plot(0,0,'go','markersize',8,...
                'MarkerFaceColor', 'g');
            
            %-----------------------------------------------------------%
            % 3. ADDITIONAL CONFIGURATION
            %-----------------------------------------------------------%
            %  3.1. SET RADIOUS
            MAX_RANGE = fix(laser.max_range);
            RR = linspace(0,MAX_RANGE,6);
            %  3.2. DISPLAY CIRCLES
            plot_circle([0;0], RR(2), 'b')
            plot_circle([0;0], RR(3), 'b')
            plot_circle([0;0], RR(4), 'b')
            plot_circle([0;0], RR(5), 'b')
            plot_circle([0;0], RR(6), 'b')
            plot_circle([0;0], laser.max_range, 'r')
            %  3.3. DISPLAY SOME LINES
            max = laser.max_range + 5;
            plot([0, max*cos(-pi/6)],   [0, max*sin(-pi/6)],  '-m')
            plot([0, max*cos(0)],       [0, max*sin(0)],      '-m')
            plot([0, max*cos(pi/4)],    [0, max*sin(pi/4)],   '-m')
            plot([0, max*cos(pi/2)],    [0, max*sin(pi/2)] ,  '-m')
            plot([0, max*cos(3*pi/4)],  [0, max*sin(3*pi/4)], '-m')
            plot([0, max*cos(pi)],      [0, max*sin(pi)],     '-m')
            plot([0, max*cos(pi+pi/6)], [0, max*sin(pi+pi/6)],'-m')
            %  3.4. SET PREVIOUS FIGURE
            figure(h_previous_fig);
        end
        %-------------- END VISUALIZATION FUNCTIONS ------------------%      
    end
    
    %------------------------------------------------------------------%
    %  PRIVATE METHODS
    %------------------------------------------------------------------%
    methods(Access=private)
        function s = sense_using_laser_normal(laser, pose, NOISE, DEBUG)
            % robot.sense_using_laser_normal Simulate the measurements of a 
            %                                laser range finder
            %
            %   S = SENSE_USING_LASER_NORMAL(POSE, NOISE, DEBUG) Simulates the
            %       measurements of the laser in the pose "POSE" using the
            %       "map" given to the object.
            %
            % OUTPUT
            %       s: Vector that contains the laser scanning [Kx1]
            %
            % INPUTS
            %      pose: Pose of the laser
            %     NOISE: Flag to indicate the insertion of noise
            %               1: The sensor measurements are noise(default)
            %               0: The sensor is noise-free
            %     DEBUG: Flag for debugging
            %
            %
            %  NOTE: In this implementation we assume that the angles of 
            %        the beams of the laser are in the range [0-180]. This 
            %        correspond to the local frame [-pi/2, pi/2])
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%     
            %  1.1. SET DEFAULT VALUE OF THE NOISE FLAG
            if(nargin < 3)
                NOISE = 1;
            end
            %  1.2. SET DEFAULT VALUE OF THE DEBUG FLAG
            if(nargin < 4)
                DEBUG = 0;
            end
            
            %-----------------------------------------------------------%
            % 2. CHECK NECCESARY PROPERTIES
            %-----------------------------------------------------------%        
            %  2.1. CHECK ANGLES OF THE LASER BEAMS
            if(laser.theta_laser(1) < -pi/2)
                error('laser.SENSE_USING_LASER_NORMAL: Min allowed angle is -pi/2')
            end
            if(laser.theta_laser(end) > pi/2)
                error('laser.SENSE_USING_LASER_NORMAL: Max allowed angle is pi/2')
            end
            %  2.2. GET PROPERTIES TO SPEED UP
            MAX_RANGE             = laser.max_range;
            KK                    = laser.K;
            THETA_LASER_SENSE     = laser.theta_laser_sense;
            SIN_THETA_LASER_SENSE = laser.sin_theta_laser_sense;
            COS_THETA_LASER_SENSE = laser.cos_theta_laser_sense;
            
            %-----------------------------------------------------------%
            % 3. SENSOR INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            s = MAX_RANGE*ones(KK,1);
            
            %-----------------------------------------------------------%
            % 4. GET A THE LOCAL MAP "map_tmp" SUCH THE POSE OF THE ROBOT
            %    IS AT THE ORIGIN AND THAT THE BEAM WITH ANGLE "0" IS 
            %    ALONG THE "x-axes"
            %-----------------------------------------------------------%
            %  4.1. SET THE TEMPORAL MAP. Note "map(i,:)=[X1 Y1 X2 Y2]"
            map_tmp = laser.map;
            %  4.2. DEFINE THE NEW MAP SUCH THAT THE LOCATION OF THE ROBOT
            %       IS AT THE  ORIGIN OF COORDINATES.
            map_tmp(:,[1 3]) = map_tmp(:,[1 3]) - pose(1);      % In the x's
            map_tmp(:,[2 4]) = map_tmp(:,[2 4]) - pose(2);      % In the y's
            
            %  4.3. ROTATE THE MAP SUCH THAT THE BEAM WITH LOCAL ANGLE "0" 
            %       COINCIDE WITH THE "x-axis"
            %   -> Modify the points "X1,Y1"
            [Ang,Radio] = cart2pol(map_tmp(:,1), map_tmp(:,2));
            Ang = Ang - pose(3) + pi/2;
            [map_tmp(:,1), map_tmp(:,2)] = pol2cart(Ang, Radio);
            %   -> Modify the points "X2,Y2"
            [Ang,Radio] = cart2pol(map_tmp(:,3), map_tmp(:,4));
            Ang = Ang - pose(3) + pi/2;
            [map_tmp(:,3),map_tmp(:,4)] = pol2cart(Ang,Radio);
            
            %-----------------------------------------------------------%
            % 5. MAIN CODE
            %   -> The idea is to test all lines that define the temporal 
            %      map, see if the line is detectable from the laser, and 
            %      using "min" select the minimum distances. Remember that 
            %      "map(i,:)=[X1 Y1 X2 Y2]"
            %-----------------------------------------------------------%
            for f = 1:size(map_tmp,1)
                % 5.1. DISPLAY DEBUG FOR DEBUGGING PURPOSES IF DESIRED
                if(DEBUG)
                    fprintf('Line: %d\n', f)
                    fprintf('  [X1 Y1 X2 Y2] = [%2.4f, %2.4f, %2.4f, %2.4f]\n',...
                        laser.map(f,:))
                    fprintf('  [x1 y1 x2 y2] = [%2.4f, %2.4f, %2.4f, %2.4f]\n',...
                        map_tmp(f,:))
                end
                
                % 5.2 COMPUTE THE EQUATION "y = mx + b" FOR THE "fth" LINE 
                %     IN THE TEMPORAL MAP. Note: "k1 = m" is the slope, and
                %     "k2 = b"
                %   -> Compute "k1"
                k1 = (map_tmp(f,4) - map_tmp(f,2))/(map_tmp(f,3) - map_tmp(f,1));
                %   -> Compute "k2" - Note that in vertical lines this is undefined
                k2 = map_tmp(f,2) - k1*map_tmp(f,1);
                
                % 5.3 COMPUTE THE MINIMUM DISTANCE FROM THE ORIGIN TO THE
                %     LINE IN THE TEMPORAL MAP
                if(abs(k1) ~= inf)
                    % The line has no infinite slope
                    dist_min = abs(k2/(1+k1^2))*norm([k1 -1]);
                else
                    % The line is a vertical line
                    dist_min = abs(map_tmp(f,1));
                end
                
                % 5.4 DISPLAY DEBUG FOR DEBUGGING
                if(DEBUG)	
                    fprintf('  k1 = %4.4f, k2 = %4.4f\n',k1,k2);
                    fprintf('  Distance = %4.4f\n', dist_min)
                end
                
                % 5.5 IF THE LINE "fth" IS IN THE FIELD OF VIEW OF THE
                %     SENSOR, COMPUTE THE BEAMS THAT IMPACT THIS LINE.
                if(dist_min < MAX_RANGE)
                    % A. COMPUTE THE ANGLE OF THE POINTS "A, B" THAT DEFINE
                    %    THE LINE IN THE LOCAL MAP
                    %   -> We use "atan2" to work in the range [-pi,pi]
                    theta_A = atan2(map_tmp(f,2),map_tmp(f,1));
                    theta_B = atan2(map_tmp(f,4),map_tmp(f,3));    
                    %   -> Make sure that "theta_B > theta_A"
                    if(theta_A > theta_B)
                        tmp = theta_A;
                        theta_A = theta_B;
                        theta_B = tmp;
                    end
                    
                    % B. IF THE FOLLOWING HAPPENS, IT IS IMPOSIBLE FOR THE LASER TO
                    %    REACH THIS WALL EVEN THOUGH IT IS IN ITS RANGE
                    if(theta_B < 0)
                        if(DEBUG)
                            fprintf('   This line is close to the laser but indetectable\n')
                            fprintf('    theta_A = %4.4f, theta_B = %4.4f\n\n', theta_A,theta_B)
                        end
                        continue;
                    end
                    
                    % C. THIS IS TO MANAGE SPECIAL CASES WHEN "theta_A < 0"
                    if(theta_A < 0)
                        if(theta_B > theta_A+pi) 
                            theta_A = theta_B;
                            theta_B = pi;
                        else
                            theta_A = 0;
                        end
                    end
                    if(DEBUG)
                        fprintf('  This line is close to the laser and detectable\n')
                        fprintf('  theta_A = %4.4f, theta_B = %4.4f\n', theta_A,theta_B)
                    end
                    
                    % D. NOW THAT WE HAVE THE ANGLES "A,B" THAT DEFINE THE SEGMENT OF
                    %    IMPACT OF THE LASER, COMPUTE THE CORRESPONDING INDEXES OF THE
                    %    VECTOR "theta_laser_sense" TO GET THE ANGLES OF THE BEAMS
                    % -> Angle "theta_A"
                    paso = (THETA_LASER_SENSE(end) - THETA_LASER_SENSE(1));
                    if(theta_A <= THETA_LASER_SENSE(1) )
                        first = 1;
                    else
                        % Matlab is 1-indexing
                        first = ceil((theta_A-THETA_LASER_SENSE(1))*(KK-1)/paso) + 1;
                    end
                    % -> Angle "theta_B"
                    if(theta_B >= THETA_LASER_SENSE(end) )
                        last = KK;
                    else
                        % 1-indexing
                        last = floor((theta_B-THETA_LASER_SENSE(1))*(KK-1)/paso) + 1;
                    end
                    %   -> Display messages
                    if(DEBUG)
                        fprintf('  first: %d, last:%d\n', first, last)
                    end
                    
                    % E. TO HANDLE SPECIAL CASES(happens when theta_laser_sense(1)~=0 
                    %    and theta_laser_sense(end)~=pi )
                    %   -> Check indexes
                    if(last < first)
                        if(DEBUG)
                            fprintf('  last < first\n')
                        end
                        continue
                    end
                    %   -> Check angles
                    if(THETA_LASER_SENSE(end) < theta_A)
                        if(DEBUG)
                            fprintf('  theta_laser_sense(end) < theta_A\n')
                        end
                        continue
                    end
                    %   -> Check angles
                    if(THETA_LASER_SENSE(1) > theta_B)
                        if(DEBUG)
                            fprintf('  theta_laser_sense(1) > theta_B\n')
                        end
                        continue
                    end
                    
                    % F. GET THE LASER DISTANCES TO THE LINE, AND UPDATE THE
                    %    SENSOR MEASUREMENTS USING THE MINIMUM DISTANCES
                    if(abs(k1) ~= inf)
                        s(first:last) = min(s(first:last),...
                            (k2./(SIN_THETA_LASER_SENSE(first:last) -...
                            k1*COS_THETA_LASER_SENSE (first:last))) );
                    else
                        s(first:last) = min(s(first:last),...
                           abs(dist_min./COS_THETA_LASER_SENSE(first:last)) );
                    end
                end
                
                if(DEBUG)
                    fprintf('\n')
                end
            end
                        
            %-----------------------------------------------------------%
            % 6. NOISE
            %-----------------------------------------------------------%
            if(NOISE)
                %  6.1. SIMULATE LOCAL MEASUREMENT NOISE (gaussian noise) 
                ind = (s~=MAX_RANGE);
                s(ind) = s(ind) + laser.std_local_noise*randn(size(s(ind)));          
                % 6.2. SIMULATE SHORT READINGS
                for k=1:KK
                    if rand() < laser.short_noise
                        s(k) = s(k)*rand();
                    end
                end
                %  6.3. SIMULATE "max-readings" FAILURES.
                for k=1:KK
                    if rand() < laser.max_noise
                        s(k) = MAX_RANGE;
                    end
                end
            end            
            
        end
        
        function s = sense_using_laser_full(laser, pose, NOISE)
            % laser.sense_using_laser_full  Simulate laser range finder 
            %
            %   S = SENSE_USING_LASER_FULL(POSE, NOISE) Simulates the laser
            %       measurements in full range 
            %
            % OUTPUT
            %       s: Vector that contains the laser scanning [Kx1]
            %
            % INPUTS
            %      pose: Pose of the laser
            %     NOISE: Flag to indicate the insertion of noise
            %               1: The sensor measurements are noise(default)
            %               0: The sensor is noise-free
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. SET DEFAULT VALUE OF THE NOISE FLAG
            if(nargin < 3)
                NOISE = 1;
            end
            
            %-----------------------------------------------------------%
            % 2. CHECK NECCESARY PROPERTIES
            %-----------------------------------------------------------%        
            %  2.1. GET PROPERTIES TO SPEED UP
            THETA_LASER = laser.theta_laser;
            MAX_RANGE   = laser.max_range;
            MAP2        = laser.map2;
            KK          = laser.K;
            SHORT_NOISE = laser.short_noise;
            MAX_NOISE   = laser.max_noise;        
            
            %----------------------------------------------------------%
            % 3. INITIAL CONFIGURATION
            %----------------------------------------------------------%
            %  3.1. GET THE LOCATION AND ORIENTATION OF THE LASER
            location = pose(1:2); 
            theta    = pose(3);
            
            %----------------------------------------------------------%
            % 4. MAIN CODE
            %----------------------------------------------------------%
            %  4.1. SCANNING INITIALIZATION
            s = MAX_RANGE*ones(KK,1);
            %  4.2. MAIN LOOP
            for i = 1:KK
                % A. DEFINE THE GEOMETRY OF BEAM "i"
                %   -> Set local "global" angle   
                angle = pi_to_pi(THETA_LASER(i) + theta);
                %   -> Set "maximum" end point
                Tr(1) = location(1) + cos(angle)*MAX_RANGE;
                Tr(2) = location(2) + sin(angle)*MAX_RANGE;
                
                % B. FIND THE INTERSECTION BETWEEN THE BEAM AND THE 
                %    WORLD LINES
                [xout, yout] = polyxpoly(MAP2(:,1),  MAP2(:,2),...
                    [location(1) Tr(1)], [location(2) Tr(2)]);
                %   -> Continue if we have skipped readings
                if(isempty(xout) )
                    continue;
                end
                
                % C. GET THE MINIMUM DISTANCE OF INTERSECTIONS
                if(size(xout,1) > 1)
                    mind = inf;
                    for k = 1:size(xout,1)
                        % Get distance to intersection points
                        d = norm([xout(k)-location(1), yout(k)-location(2)]);
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
                        
            %----------------------------------------------------------%
            % 5. ADD NOISE IF DESIRED
            %----------------------------------------------------------%
            if(NOISE)
                % 5.1. SIMULATE LOCAL MEASUREMENT NOISE (gaussian noise)
                ind = s~=MAX_RANGE;
                s(ind) = s(ind) + laser.std_local_noise*randn(size(s(ind)));
                % 5.2. SIMULATE SHORT READINGS
                for k=1:KK
                    if(rand() < SHORT_NOISE)
                        s(k) = s(k)*rand();
                    end
                end
                % 5.3. SIMULATE "max-readings" FAILURES
                for k=1:KK
                    if(rand() < MAX_NOISE)
                        s(k) = MAX_RANGE;
                    end
                end
            end
        end
    end
    
    %------------------------------------------------------------------%
    %  STATIC METHODS
    %------------------------------------------------------------------%
    methods(Static)
        function credits()
            % laser.credits Credits
            disp('---------------------------------')
            disp('--- About the "laser" library ---')
            disp('---------------------------------')
            disp(' Author: Ivan A. Calle Flores       ')
            disp(' e-mail: ivan.calle.flores@gmail.com')
        end
    end
end

