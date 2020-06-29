% OCC_GRID Occupancy grid map
%
%   This class implements the methods and properties to handle the
%   occupancy grid maps
classdef occ_grid < handle
    %------------------------------------------------------------------%
    %  DECLARE THIS PROPERTIES SUCH THAT
    %   - Can only be accesed inside the class
    %   - Can be read from anywhere
    %------------------------------------------------------------------%
    properties (SetAccess=protected, GetAccess=public)
        %------------------------------------------------------------%
        % MAIN PROPERTIES
        %------------------------------------------------------------%
        grid_map        % Occupancy grid map
        log_grid_map
        IS_GRID         % Flag to check the existence of a grid
        MODE_PLOTTING   % 'NORMAL' cells in range [0-1] - localization 
                        % 'LAYERS' cells are [0,1,2,3, etc] - planer, etc
        occupied_thresh % occupied_threshold 
        free_thresh     % free_threshold
        GRID_FILE       % Name of the '.yaml' file that holds the grid
                        % Must be the full path
        
        %------------------------------------------------------------%
        % FEATURES OF THE GRID MAP
        %------------------------------------------------------------%
        grid_xmin       % Coordinates of the Bottom left corner(image)
        grid_ymin       
        grid_xmax       % Coordinates of the Top right corner(image)
        grid_ymax      
        grid_deltax     % Delta of the map(resolution of the grid)
        grid_deltay
        grid_rows       % Number of rows and columns of the grid map
        grid_cols       
        grid_xx         % Vector of representative coordinates (centers)
        grid_yy     
        grid_XXX        % Matrix of representative coordinates (centers)
        grid_YYY        
        grid_TYPE       % Type of grid_map (1:REAL / 0:IDEAL)
        
        %------------------------------------------------------------%
        % HANDLES TO DO THE ANIMATIONS
        %------------------------------------------------------------%
        % MAIN HANDLES
        h_grid_fig      % Handle of the grid figure
        h_grid_map      % handle of the grid map
        h_grid_fig_mindistances
        % ADDITIONAL HANDLES TO DO SIMULATIONS
        h_robot         % To plot the robot
        h_laser
        h_vehicle       % To plot the real robot
        h_mu            % To plot a Gaussian
        h_P
        
        %------------------------------------------------------------%
        % PARAMETERS TO COMPUTE THE MINIMUM DISTANCE TO A CELL
        %------------------------------------------------------------%
        MAXIMUM_DISTANCE            % Maximum minimum distance
        MAXIMUM_DISTANCE2
        index_offset_square_xx      % Number of equivalent indexes of the
        index_offset_square_yy      % square of search. "ceil(L/delta)"
        
        %------------------------------------------------------------%
        % MATRIX OF MINIMUM DISTANCES
        %------------------------------------------------------------%
        grid_map_min_distance       % Matrix of min distances
        grid_map_min_distance2      % Matrix of min squared distances
        grid_min_distance_xbar      % Matrix of coordinates of the closest
        grid_min_distance_ybar      % grid cell for each grid
        
        %------------------------------------------------------------%
        % FOR PLANNING, REPLANNING, AND MAPPING
        %------------------------------------------------------------%
        % INTERNAL ROBOT
        my_robot            % Object of the class 'robot'
        NAME_ROBOT          %'R2D2-00', 'R2D2-R1,', 'R2D2-R2,' etc.
        % TO MANAGE THE ROBOT'S BODY
        XX_robot            % Coordinates of the robot's body [Nx1] when
        YY_robot            % the pose of the robot is [0;0;0]
        IDX_robot           % Indexes of the robot's body in the grid map
        IDY_robot           % matrix [Nx1]
    end
    
    %------------------------------------------------------------------%
    % PUBLIC METHODS
    %------------------------------------------------------------------%
    methods
        %---------------------- CONSTRUCTOR --------------------------%
        function this = occ_grid(grid_file)
            % occ_grid.occ_grid  Constructor function
            %
            %   OCC_GRID() returns a valid "occ_grid" object. 
            %
            %   OCC_GRID(GRID_MAP) returns a valid "occ_grid" object. It
            %   also set the grid map
            
            %----------------------------------------------------------%
            % 1. SET THE INITIAL GRID MAP IF DESIRED
            %----------------------------------------------------------%
            if(nargin>=1)
                %-------------------------------------------------------%
                % A. CHECK THE ARGUMENT
                %-------------------------------------------------------%
                if(~ischar(grid_file))
                    error('OCC_GRID: First argument must be a string')
                end
                
                %-------------------------------------------------------%
                % B. SET THE GRID MAP
                %-------------------------------------------------------%
                this.set_grid_map(grid_file);
                this.GRID_FILE = grid_file;
                this.MODE_PLOTTING = 'NORMAL';   % Range [0-1] LOCALIZATION
            else
                % IF THERE IS NO GRID
                this.MODE_PLOTTING = 'NORMAL';
                this.IS_GRID = 0;
            end
        end
        %-------------------- END CONSTRUCTOR ------------------------%
        
        
        %-------------- OCCUPANCY GRID BASIC FUNCTIONS ---------------%
        function set_grid_map(this, yaml_file)
            % occ_grid.set_grid_map Set the grid map of the map
            %
            %   SET_GRID_MAP(YAML_FILE) Sets the property grid_map and
            %   its attributes from the ROS yaml format
            %
            % INPUT
            %   YAML_FILE: Yaml ROS file
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['occ_grid.set_grid_map:', ...
                    'Insuficient number of parameters'];
                error(str)
            end
            %  1.2. CHECK THE ARGUMENT "grid_map"
            if(~ischar(yaml_file))
                str = ['occ_grid.set_grid_map:', ...
                    'Argument must be a string'];
                error(str)
            end
            %  1.3. CHECK EXISTENCE OF THE FILE
            if(exist(yaml_file, 'file') ~= 2)
                str = ['occ_grid.set_grid_map:', ...
                    'The yaml file does not exist: ' yaml_file];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. LOAD THE YAML FILE AND SET THE GRID MAP
            %-----------------------------------------------------------%
            %  2.1. LOAD THE FILE
            ys = ReadYaml(yaml_file);
            %  2.2. READ DE GRID MAP FROM THE IMAGE
            %   -> Read the image
            img = imread(ys.image);
            if(size(img,3)>1) % Converting from RGB to gray
                img = rgb2gray(img);
            end
            %   -> Convert to double [0-1] and flip
            img = double(img)/255;
            img = flipud(img);
            %   -> Converto to our MATLAB format 1:Occupied, 0:Free
            img = -img + 1;
            %  2.3. READ THE PROPERTIES OF THE GRID
            %   -> Origin (of the lower-left pixel in the map)
            xmin = ys.origin{1};
            ymin = ys.origin{2};
            %   -> Resolution of the map (meters/pixel) 
            dd = ys.resolution;
            %   -> Threshold for probability of occupation and free
            p_occ  = ys.occupied_thresh;
            p_free = ys.free_thresh;
            img(img > p_occ)  = 1;
            img(img < p_free) = 0;
            %   -> Read the dimensions of the grid (number of grid cells)
           	[GRID_ROWS, GRID_COLS] = size(img);
            
            %-----------------------------------------------------------%
            %  3. SET OCCUPANCY GRID MAP
            %-----------------------------------------------------------%
            %  3.1. SET MAIN PROPERTIES
            this.grid_map        = img;
            this.IS_GRID         = 1;
            this.occupied_thresh = p_occ;
            this.free_thresh     = p_free;
            this.GRID_FILE       = yaml_file;
            %  3.2. SET GEOMETRIC PROPERTIES
            this.grid_xmin = xmin;
            this.grid_ymin = ymin;
            this.grid_rows = GRID_ROWS;
            this.grid_cols = GRID_COLS;
            %  3.3. SET TYPE OF MAP (0:ideal/1:real)   
            NN = img < p_occ & img > p_free;    
            if(sum(sum(NN)) >= 1)
                % If at lest there's one unknow grid cell the map is real
                this.grid_TYPE = 1;
            else
                this.grid_TYPE = 0;
            end
            %  3.4. SET LOG GRID MAP
            this.log_grid_map = log(this.grid_map./(1-this.grid_map));
            
            %-----------------------------------------------------------%
            % 4. SET OTHER PROPERTIES OF THE GRID MAP
            %-----------------------------------------------------------%
            %  4.1. COORDINATES OF THE TOP RIGHT CORNER
            xmax = xmin + GRID_COLS*dd;
            ymax = ymin + GRID_ROWS*dd;
            %  4.2. VECTOR OF REPRESENTATIVE COORDINATES OF THE CELLS
            indx = 1:GRID_COLS;
            indy = 1:GRID_ROWS;
            xx = (indx-1)*dd + xmin + dd/2;
            yy = (indy-1)*dd + ymin + dd/2;
            %  4.3. SET DELTAS (IN ROS THESE ARE EQUAL)
            GRID_DELTAX  = dd;
            GRID_DELTAY  = dd;
            %  4.4. SET ATTRIBUTES
            this.grid_xmax   = xmax;
            this.grid_ymax   = ymax;
            this.grid_xx     = xx;
            this.grid_yy     = yy;
            this.grid_deltax = GRID_DELTAX;
            this.grid_deltay = GRID_DELTAY;
            
            %-----------------------------------------------------------%
            % 5. SET MATRIX OF REPRESENTATIVE COORDINATES OF EACH GRID
            %-----------------------------------------------------------%
            %  5.1. COMPUTE THE REPRESENTATIVE COORDINATES 
            GRID_XXX = zeros(GRID_ROWS, GRID_COLS);
            GRID_YYY = zeros(GRID_ROWS, GRID_COLS);
            for idy = 1:GRID_ROWS
                for idx = 1:GRID_COLS
                    GRID_XXX(idy,idx) = (idx-1)*GRID_DELTAX + ...
                        xmin + GRID_DELTAX/2;
                    GRID_YYY(idy,idx) = (idy-1)*GRID_DELTAY + ...
                        ymin + GRID_DELTAY/2;
                end
            end
            %  5.2. SET THE MATRIX
            this.grid_XXX = GRID_XXX;
            this.grid_YYY = GRID_YYY;
        end
        
        function save_grid_map(this, yaml_file, path)
            % occ_grid.save_grid_map Save the 'grid_map' in a file
            %
            %   SAVE_GRID_MAP Save the grid map and its properties using
            %   the ROS 'yaml' format.
            %
            % INPUTS
            %   name: Name of the file
            %   path: 
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. CHECK THE ARGUMENT "yaml_file"
            if(~ischar(yaml_file))
                error('occ_grid.save_grid_map: Argument must be a string')
            end
            
            %-----------------------------------------------------------%
            % 2. PROCESS THE GRID MAP
            %-----------------------------------------------------------%
            %  2.1. GET THE GRID
            GRID_MAP = this.grid_map;
            %  2.2. PROCESS GRID MAP
            %   -> Invert (0 to 1 and viceversa)
            GRID_MAP = -GRID_MAP + 1;
            %   -> Flip the grid (so that the image appears ok in ROS)?
            GRID_MAP = flipud(GRID_MAP);
            %   -> Convert to uint8
            img_out = uint8(GRID_MAP*255);
            %   -> Show image 
            figure('Name','ROS MAP')
            imshow(img_out)
            %  2.3. GET NAME OF THE FILE WITHOUT THE EXTENSION '.yaml'
            grid_file = yaml_file(1:end-5);
            
            %-----------------------------------------------------------%
            % 3. SET 'yaml' STRUCT
            %-----------------------------------------------------------%
            ys.image      = [grid_file '.png'];
            ys.resolution = this.grid_deltax;
            ys.origin     = {this.grid_xmin  this.grid_ymin  0};
            ys.negate     = 0;
            ys.occupied_thresh = this.occupied_thresh;
            ys.free_thresh     = this.free_thresh;
            
            %-----------------------------------------------------------%
            % 4. SAVE DATA
            %-----------------------------------------------------------%
            % SET NAMES OF THE FILES
            if(nargin==3)
                full_name_yaml = [path grid_file '.yaml'];
                full_name_img  = [path ys.image];
            else
                full_name_yaml = [grid_file '.yaml'];
                full_name_img  = ys.image;
            end
            % WRITE THE IMAGE
            imwrite(img_out, full_name_img)
            % WRITE THE ".yaml"
            WriteYaml(full_name_yaml, ys);
            fprintf('occ_grid.save_grid_map: Saving %s grid map\n', full_name_yaml);
        end
        
        function set_grid_map_matrix(this, grid_map_matrix)
            % occ_grid.set_grid_map_matrix
            this.grid_map = grid_map_matrix;
        end
        
        function set_log_grid_map_matrix(this, log_grid_map_matrix)
            % occ_grid.set_grid_map_matrix
            this.log_grid_map = log_grid_map_matrix;
        end
        
        function set_grid_map_thresholds(this, p_occ, p_free)
            % occ_grid.set_grid_cell_thresholds
            this.occupied_thresh = p_occ;
            this.free_thresh     = p_free;
        end
        
        function display_grid_properties(this)
           % occ_grid.display_properties Display properties
           fprintf(' Properties of ''%s'' \n', inputname(1))
           fprintf('    yaml_file: ''%s'' \n', this.GRID_FILE)
           fprintf('    grid_xmin: %2.4f \n', this.grid_xmin)
           fprintf('    grid_xmax: %2.4f \n', this.grid_xmax)
           fprintf('    grid_ymin: %2.4f \n', this.grid_ymin)
           fprintf('    grid_ymax: %2.4f \n', this.grid_ymax)
           fprintf('    grid_deltax: %2.4f \n', this.grid_deltax)
           fprintf('    grid_deltay: %2.4f \n', this.grid_deltay)
           fprintf('    grid_rows: %d \n', this.grid_rows)
           fprintf('    grid_cols: %d \n', this.grid_cols)
           fprintf('    grid_TYPE: %d \n', this.grid_TYPE)
        end
        
        function negate_grid_map(this)
            this.grid_map = 1- this.grid_map;
        end
        
        function set_new_occupancy_grid_map(this,...
                occ_grid_map_dimensions)
            % mapper.set_new_occupancy_grid_map Set 
            %
            %   SET_OCCUPANCY_GRID_MAP_DIMENSIONS Set the dimensions of the
            %   grid map to be build in the mapping mode
            % 
            % INPUTS
            %   occ_grid_map_dimensions: Structure that has the dimensions
            %                            of the grid map to be built
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. GET THE FEATURES OF THE GRID MAP
            GRID_XMIN   = occ_grid_map_dimensions.xmin;
            GRID_YMIN   = occ_grid_map_dimensions.ymin;
            GRID_XMAX   = occ_grid_map_dimensions.xmax;
            GRID_YMAX   = occ_grid_map_dimensions.ymax;
            GRID_DELTAX = occ_grid_map_dimensions.deltax;
            GRID_DELTAY = occ_grid_map_dimensions.deltay;
            
            %-----------------------------------------------------------%
            % 2. GRID MAP INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1 SET DIMENSIONS GIVEN BY THE USER
            this.grid_xmin   = GRID_XMIN;
            this.grid_ymin   = GRID_YMIN;
            this.grid_xmax   = GRID_XMAX;
            this.grid_ymax   = GRID_YMAX;
            this.grid_deltax = GRID_DELTAX;
            this.grid_deltay = GRID_DELTAY;
            %  2.2 SET DIMENSIONS OF THE GRID
            %   -> Set number of columns
            cols = (GRID_XMAX - GRID_XMIN)/GRID_DELTAX;
            GRID_COLS = ceil(cols);      % Round towards +inf
            this.grid_cols = GRID_COLS;
            %   -> Set number of rows
            rows = (GRID_YMAX - GRID_YMIN)/GRID_DELTAY;
            GRID_ROWS = ceil(rows);      % Round towards +inf
            this.grid_rows = GRID_ROWS;
            %  2.3 SET "representative" COORDINATES OF EACH CELL(the centers)
            %   -> Get the 1-based index of each cell
            indx = 1:GRID_COLS;
            indy = 1:GRID_ROWS; 
            %   -> Set the coordinates
            this.grid_xx = (indx-1)*GRID_DELTAX + GRID_XMIN + GRID_DELTAX/2;
            this.grid_yy = (indy-1)*GRID_DELTAY + GRID_YMIN + GRID_DELTAY/2;
            
            %-----------------------------------------------------------%
            % 3. SET INITIAL VALUE OF THE GRID MAP AS UNKNOWN
            %-----------------------------------------------------------%
            %  3.1. SET GRID MAP
            this.grid_map = 0.5*ones(GRID_ROWS, GRID_COLS);
            %  3.2. SET THE LOG OF THE GRID MAP (to zeros)
            this.log_grid_map = log(this.grid_map./(1-this.grid_map));
            
            %-----------------------------------------------------------%
            % 4. SET REPRESENTATIVE COORDINATES OF EACH GRID CELL
            %-----------------------------------------------------------%
            %  4.1. COMPUTE THE REPRESENTATIVE COORDINATES 
            XTEST_GRID = zeros(GRID_ROWS, GRID_COLS);
            YTEST_GRID = zeros(GRID_ROWS, GRID_COLS);
            for idy = 1:GRID_ROWS           % Run in Rows
                for idx = 1:GRID_COLS       % Run in columns
                    XTEST_GRID(idy, idx) = (idx-1)*GRID_DELTAX + ...
                        GRID_XMIN + GRID_DELTAX/2;
                    YTEST_GRID(idy, idx) = (idy-1)*GRID_DELTAY + ...
                        GRID_YMIN + GRID_DELTAY/2;
                end
            end
            %  4.2. SET THE PROPERTY
            this.grid_XXX = XTEST_GRID;
            this.grid_YYY = YTEST_GRID;
            
            %-----------------------------------------------------------%
            % 5. SET FLAG OF GRID
            %-----------------------------------------------------------%
            this.IS_GRID = 1;
        end
        %------------- END OCCUPANCY GRID BASIC FUNCTIONS ------------%    
        
                
        %-------------------- PLOTING FUNCTIONS ----------------------%
        function set_mode_plotting(this, mode)
            % occ_grid Set model plotting
            switch(mode)
               case {'NORMAL', 'LAYERS'}
                   this.MODE_PLOTTING = mode;
                otherwise
                    str = ['occ_grid.set_mode_plotting:'...
                        'Undefined name for MODE_PLOTTING'];
                    error(str);
            end
        end
        
        function create_figure(this, world)
            % occ_grid.create_figure Create a figure to plot the grid map
            %
            %   CREATE_FIGURE() Create a figure to plot the grid map
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. DEFAULT VALUES
            VISIBLE  = 'on';
            TITLE    = 'OCCUPANCY GRID MAP';
            POSITION = [10 50 500 400];
            %  1.2. PROCESS USER CONFIGURATION
            if(nargin ==2)
                % Set visibility
                if(isfield(world, 'visibility'))
                    VISIBLE = world.visibility;
                end
                % Set title
                if(isfield(world, 'title'))
                    TITLE = world.title;
                end
                % Set position
                if(isfield(world, 'position'))
                    POSITION = world.position;
                end
            end
            
            %-----------------------------------------------------------%
            % 2. CREATE THE MAIN FIGURE
            %-----------------------------------------------------------%
            %  2.1. CREATE A FIGURE
            h_fig = figure('Visible',VISIBLE);
            %  2.2. SET PROPERTIES
            set(h_fig,'position', POSITION, 'Name', TITLE,...
                'color', [211 208 200]/255);
            %  2.3. ADDICIONAL PROPERTIES
            set(gca, 'Box', 'on')
            hold on
            xlabel('x(m)')
            ylabel('y(m)')
            
            %-----------------------------------------------------------%
            % 3. SAVE HANDLE OF THE FIGURE
            %-----------------------------------------------------------%
            this.h_grid_fig = h_fig;
        end
        
        function plot_occupancy_grid(this, world)
            % occ_grid.plot_occupancy_grid Plot the occupancy grid map
            %
            %   PLOT_OCCUPANCY_GRID() Plots the occupancy grid map in
            %   the figure of the grid. Also create handles to plot the
            %   robot, the gaussian, and the real vehicle
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. MAKE SURE THE GRID IS DEFINED
            if(isempty(this.grid_map))
                str = ['occ_grid.plot_occupancy_grid:',...
                    'The grid map is not defined'];
                error(str)
            end
            %  1.2. MAKE SURE THE FIGURE IS DEFINED
            if(isempty(this.h_grid_fig))
                if(nargin==2)
                    this.create_figure(world);
                else
                    this.create_figure();
                end
            end
            %  1.3. GET GRID_MAP
            GRID_MAP = this.grid_map;
            
            %-----------------------------------------------------------%
            % 2. PLOT THE ROBOT'S BODY SPACE IF REQUIRED
            %-----------------------------------------------------------%
            if(~isempty(this.IDX_robot))
                % Get indexes of the robot body
                IDX_ROBOT = this.IDX_robot;     % [Nx1]
                IDY_ROBOT = this.IDY_robot;     % [Nx1]
                % Set this space as occupied
                for ind = 1:length(IDX_ROBOT)
                    r = IDY_ROBOT(ind);
                    c = IDX_ROBOT(ind);
                    GRID_MAP(r,c) = 1;
                end
            end
            
            %-----------------------------------------------------------%
            % 3. PLOT THE OCCUPANCY GRID
            %-----------------------------------------------------------%
            if(isempty(this.h_grid_map))
                % WHEN WE PLOT THE GRID MAP THE FIRST TIME
                %-----------------------------------------------------%
                %  A. SET THE GRID FIGURE AS CURRENT
                %-----------------------------------------------------% 
                h_previous_fig = gcf; 
                figure(this.h_grid_fig);
                
                %-----------------------------------------------------%
                %  B. PLOT THE GRID MAP AND SAVE ITS HANDLE
                %-----------------------------------------------------%
                if(strcmp(this.MODE_PLOTTING, 'NORMAL'))
                    % In this mode we invert the grid cells in order to 
                    % print the map as presented in the books
                    this.h_grid_map = imagesc(1-GRID_MAP,...
                        'XData',this.grid_xx, 'YData', this.grid_yy);
                elseif(strcmp(this.MODE_PLOTTING, 'MAPPER') || strcmp(this.MODE_PLOTTING,'LOCAL-MAPPER') )
                    % We print the map as it is.
                    this.h_grid_map = imagesc(GRID_MAP,...
                        'XData',this.grid_xx, 'YData', this.grid_yy);
                else
                    str = ['occ_grid.plot_occupancy_grid:',...
                    'The MODE is not valid'];
                    error(str)
                end
                %   -> Set colormap
                colormap(gray)
                %   -> This is in order y-axes in ascending order
                set(gca, 'Ydir', 'normal');
                %   -> Set axes
                xlabel('x(m)')
                ylabel('y(m)')
                hold on
                
                %-----------------------------------------------------%
                %  C. SET THE CORRECT VALUES OF THE AXES' RANGES
                %-----------------------------------------------------% 
                %   -> Get min and max along the x-axes
                xmin_map = this.grid_xmin;
                xmax_map = this.grid_xmax;
                %   -> Get min and max along the y-axes
                ymin_map = this.grid_ymin;
                ymax_map = this.grid_ymax;
                %   -> Set correct axis
                axis([xmin_map   xmax_map  ymin_map  ymax_map])
                
                %-----------------------------------------------------%
                % D. CREATE ADDITIONAL HANDLES
                %-----------------------------------------------------%
                %  D.1. HANDLES TO PLOT THE ROBOT
                handle_robot    = zeros(1,2);
                handle_robot(1) = line(0,0,'LineWidth', 2);
                handle_robot(2) = line(0,0,'LineWidth', 2);
                this.h_robot    = handle_robot;
                %  D.2. HANDLES TO PLOT A GAUSSIAN
                handle_mu    = zeros(1,2);
                handle_mu(1) = line(0,0,'LineWidth', 2);
                handle_mu(2) = line(0,0,'LineWidth', 2);
                this.h_mu    = handle_mu; 
                this.h_P     = plot(0,0,'b');
                %  D.3. HANDLES TO PLOT THE REAL ROBOT
                if(isempty(this.NAME_ROBOT)==0)
                    this.set_vehicle_handles();
                end
                
                %-----------------------------------------------------%
                %  E. SET PREVIOUS FIGURE
                %-----------------------------------------------------%
                figure(h_previous_fig);
            else
                % THIS IS USED WHEN WE WANT TO PLOT THE GRID AGAIN
                if(strcmp(this.MODE_PLOTTING, 'NORMAL'))
                    set(this.h_grid_map, 'CData', 1-GRID_MAP)
                elseif(strcmp(this.MODE_PLOTTING, 'MAPPER') || strcmp(this.MODE_PLOTTING,'LOCAL-MAPPER') )
                    set(this.h_grid_map, 'CData', GRID_MAP)
                else
                    str = ['occ_grid.plot_occupancy_grid:',...
                    'The MODE is not valid'];
                    error(str)
                end
            end
        end
        
        function set_vehicle_handles(this)
            % occ_grid.set_vehicle_handles Set handles to plot the vehicle
            %
            %   SET_VEHICLE_HANDLES Set handles to plot the real robot
            %   in the grid figure
            
            %-----------------------------------------------------------%
            % 1. SET THE HANDLES ACCORDING TO THE ROBOT (to the current axes)
            %-----------------------------------------------------------%
            switch(this.NAME_ROBOT)
                case {'R2D2-00', 'R2D2-01'}
                    % Matrix to save the handles
                    handle_car = zeros(1,6);
                    % Handles to fill "2 wheels + 1 box + 1 small wheel"
                    for f=1:4
                        handle_car(1,f) = fill(1, 1,'w');
                    end
                    % Handles to draw 2 lines(local reference frame)
                    handle_car(1,5) = line(0,0,'LineWidth', 2);
                    handle_car(1,6) = line(0,0,'LineWidth', 2);
                case {'R2D2-R1', 'R2D2-R2'}
                    %   -> Matrix to save the handles
                    handle_car = zeros(1,6);
                    %   -> Handles to fill "2 boxes + 2 wheels"
                    for f=1:4
                        handle_car(1,f) = fill(1, 1,'w');
                    end
                    %   -> Handles to draw lines(local reference frame)
                    handle_car(1,5) = line(0,0,'LineWidth', 3);
                    handle_car(1,6) = line(0,0,'LineWidth', 3);
                otherwise
                    str = ['occ_grid.set_vehicle_handles:'...
                        'Undefined name of robot'];
                    error(str);
            end
            
            %-----------------------------------------------------------%
            % 2. SAVE HANDLES
            %-----------------------------------------------------------%
            this.h_vehicle = handle_car;
        end
        
        function set_laser_handles(this, K)
            % occ_grid.set_laser_handles Set handles to plot the laser
            
            %-----------------------------------------------------------%
            % A. SET THE GRID FIGURE AS CURRENT
            %-----------------------------------------------------------% 
            h_previous_fig = gcf; 
            figure(this.h_grid_fig);
            
            %-----------------------------------------------------------%
            % B. SET HANDLES
            %-----------------------------------------------------------%
            handle_laser = zeros(1,K+1);
            for f=1:K
                handle_laser(1,f) = line(0,0,'Color','m');
            end
            handle_laser(K+1) = plot(0,0,'+k', 'MarkerSize', 5,...
                'LineWidth',2);
            this.h_laser = handle_laser;
            
            %-----------------------------------------------------------%
            % C. RETURN TO PREVIOUS FIGURE
            %-----------------------------------------------------------%
            figure(h_previous_fig);
        end
        
        function create_figure_mindistances(this)
            % occ_grid.create_figure_mindistances
            %
            %   CREATE_FIGURE() Create a figure to plot the minimum
            %   distances
            
            %-----------------------------------------------------------%
            % 1. CREATE THE FIGURE
            %-----------------------------------------------------------%
            h_fig = figure;
            set(h_fig,'position',[10 200 500 500],'Name','MINIMUM DISTANCES',...
                'color',[211 208 200]/255,'Visible','on');
            this.h_grid_fig_mindistances = h_fig;
        end
        
        function plot_matrix_min_distance(this, DISTANCES)
            % occ_grid.plot_matrix_min_distance Plot the matrix of minimum
            %                                   distances
            %
            %   PLOT_MATRIX_MIN_DISTANCE Plots the matrix of "min distances"
            %   using the function "imshow"
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%           
            %  1.1. MAKE SURE THE MATRIX IS DEFINED
            if(isempty(this.grid_map_min_distance))
                str = ['occ_grid.plot_matrix_min_distance:', ...
                    'The matrix of min distances is not defined'];
                error(str)
            end
            %  1.2. MAKE SURE THE FIGURE IS DEFINED
            if(isempty(this.h_grid_fig_mindistances))
                this.create_figure_mindistances();
            end
            
            %-----------------------------------------------------------%
            % 2. PLOT THE MATRIX
            %-----------------------------------------------------------%
            figure(this.h_grid_fig_mindistances)
            if(nargin==1)
                imagesc(this.grid_map_min_distance, 'XData',this.grid_xx,...
                    'YData', this.grid_yy);
            else
                imagesc(DISTANCES, 'XData',this.grid_xx,...
                    'YData', this.grid_yy);
            end
            colorbar
            pause(0.01)
            set(gca, 'Ydir', 'normal');
            xlabel('x(m)')
            ylabel('y(m)')
        end     
        %------------------ END PLOTING FUNCTIONS --------------------%
        
        
        %---------- TO MANAGE THE INDEXES OF THE GRID CELLS ----------%
        function [IDX, IDY] = compute_index_from_location(this, xtest, ytest)
            % occ_grid.compute_index_from_location Compute the index in the
            %                                      grid for a given location
            %
            %   COMPUTE_INDEX_FROM_LOCATION(XTEST, YTEST) Compute the index
            %   of the given location in the grid map.
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 3)
                str = ['occ_grid.compute_index_from_location:',...
                    'Insuficient number of parameters'];
                error(str);
            end
            %  1.2. MAKE SURE THE GRID MAP IS DEFINED
            if(isempty(this.grid_map))
                str = ['occ_grid.compute_index_from_location:',...
                    'The grid map is not defined'];
                error(str);
            end
            
            %-----------------------------------------------------------%
            % 2. MAIN CODE
            %-----------------------------------------------------------%
            %  2.1. COMPUTE INDEXES
            IDX = ceil((xtest-this.grid_xmin)/this.grid_deltax);
            IDY = ceil((ytest-this.grid_ymin)/this.grid_deltay);
            %  2.2. CHECK "bottom-left" CORNER
            IDX(IDX==0) = 1; 
            IDY(IDY==0) = 1;
        end
        
        function [xtest_grid, ytest_grid] = compute_grid_location_from_index(...
                this, idx, idy)
            % occ_grid.compute_grid_location_from_index Computes location
            %                                           from index
            %
            %   COMPUTE_GRID_LOCATION_FROM_INDEX(IDX,IDY) computes the
            %   representative location for the given indexes.
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            % 1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 3)
                str = ['occ_grid.compute_grid_location_from_index:'...
                    'Insuficient number of parameters'];
                error(str);
            end
            %  1.2. MAKE SURE THE GRID MAP IS DEFINED
            if(isempty(this.grid_map))
                str = ['occ_grid.compute_grid_location_from_index:'...
                    'The grid map is not defined'];
                error(str);
            end
            %  1.3. GET SOME PROPERTIES
            GRID_DELTAX = this.grid_deltax;
            GRID_DELTAY = this.grid_deltay;
            GRID_XMIN   = this.grid_xmin;
            GRID_YMIN   = this.grid_ymin;
            
            %-----------------------------------------------------------%
            % 2. MAIN CODE
            %-----------------------------------------------------------%
            xtest_grid = (idx-1).*GRID_DELTAX + GRID_XMIN + GRID_DELTAX/2;
            ytest_grid = (idy-1).*GRID_DELTAY + GRID_YMIN + GRID_DELTAY/2;
        end
        
        function [x_box, y_box] = compute_box_from_location(this,...
                xtest, ytest)
            % occ_grid.compute_box_from_location Computes the grid cell
            %       
            %   COMPUTE_BOX_FROM_LOCATION(XTEST, YTEST) computes the coord.
            %   of the grid cell that contains the given location
            %
            %  NOTE: If the location is beyond limits this function return
            %        empty matrices
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 3)
                str = ['occ_grid.compute_box_from_location:'...
                    'Insuficient number of parameters'];
                error(str);
            end
            %  1.2. MAKE SURE THE GRID MAP IS DEFINED
            if(isempty(this.grid_map))
                str = ['compute_box_from_location:'...
                    'The grid map is not defined'];
                error(str);
            end
            %  1.3. GET PROPERTIES
            GRID_DELTAX = this.grid_deltax;
            GRID_DELTAY = this.grid_deltay;
            GRID_XMIN   = this.grid_xmin;
            GRID_YMIN   = this.grid_ymin;
            
            %-----------------------------------------------------------%
            % 2. COMPUTE THE INDEXES[x,y] OF THE TEST POSE IN THE GRID
            %-----------------------------------------------------------%
            %  2.1 GET THE INDEXES(1-based)
            idx = ceil((xtest-GRID_XMIN)/GRID_DELTAX);
            idy = ceil((ytest-GRID_YMIN)/GRID_DELTAY);
            %   -> Check for bottom left corner
            if(idx==0)
                idx = idx + 1;
            end
            if(idy==0)
                idy = idy + 1;
            end
            % 2.2. CHECK THE INDEXES
            if(idx < 0 || idy < 0)
                % Return empty matrices
                x_box = []; 
                y_box = [];
                return;
            elseif(idx > this.grid_cols || idy > this.grid_rows)
                % Return empty matrices
                x_box = []; 
                y_box = [];
                return;
            end
            
            %-----------------------------------------------------------%
            % 3. COMPUTE THE REPRESENTATIVE COORDINATES OF THE GRID
            %   -> We choose the center of the grid as representative
            %-----------------------------------------------------------%
            xtest_grid = (idx-1)*GRID_DELTAX + GRID_XMIN + GRID_DELTAX/2;
            ytest_grid = (idy-1)*GRID_DELTAY + GRID_YMIN + GRID_DELTAY/2;
            
            %-----------------------------------------------------------%
            % 4. MAIN CODE
            %-----------------------------------------------------------%
            x_box = [xtest_grid-0.5*GRID_DELTAX  xtest_grid+0.5*GRID_DELTAX ...
                xtest_grid+0.5*GRID_DELTAX  xtest_grid-0.5*GRID_DELTAX];
            y_box = [ytest_grid+0.5*GRID_DELTAY  ytest_grid+0.5*GRID_DELTAY ...
                ytest_grid-0.5*GRID_DELTAY  ytest_grid-0.5*GRID_DELTAY];
        end
        %-------------------------- END  -----------------------------%
        
        
        %--------------- MINIMUM DISTANCES FUNCTIONS -----------------%
        function set_maximum_distance(this, L)
            % occ_grid.set_maximum_distance
            %
            %   SET_MAXIMUM_DISTANCE Set the maximum distance to search
            %   for occupied cells
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(L<0)
                str = ['occ_grid.set_maximum_distance:',...
                    'L must be greater than zero'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. SET PARAMETERS OF THE SQUARE OF SEARCH
            %-----------------------------------------------------------%
            %  2.1. SET SIZE OF THE SQUARE OF SEARCH
            this.MAXIMUM_DISTANCE  = L;
            this.MAXIMUM_DISTANCE2 = L^2;
            %  2.2. EQUIVALENT NUMBER OF CELLS OF SEARCH
            this.index_offset_square_xx = ceil(L/this.grid_deltax);  
            this.index_offset_square_yy = ceil(L/this.grid_deltay);  
        end
        
        function [dist2, xbar, ybar, xtest_grid, ytest_grid] = ...
                search_closest_grid_cell(this, xtest, ytest)
            % occ_grid.search_closest_grid_cell
            %
            %   SEARCH_CLOSEST_GRID_CELL(XTEST, YTEST) Computes the minimum
            %   "squared" distance to an occupied grid cell.
            %
            % OUTPUTS
            %   dist2: There are three cases
            %           -> d = Minimum squared distance
            %           -> d = 0 (If the test cell is OCCUPIED)
            %           -> d = MAXIMUM_DISTANCE2 (If out of range)
            %   xbar: Coordinates (center) of the closest grid cell
            %           -> NaN if not found a closest grid
            %           -> If "d=0"  ->  xbar=xtest_grid
            %   x_test_grid: Coordinates corresponding to the representative
            %                grid of the test point
            %
            % INPUTS
            %   xtest: Coordinate x of the test point
            %   ytest: Coordinate y of the test point
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. MAKE SURE THE SIZE OF THE SQUARE OF SEARCH IS DEFINED
            if(isempty(this.MAXIMUM_DISTANCE))
                str = ['occ_grid.search_closest_grid_cell:',...
                    'The MAXIMUM_DISTANCE is not defined'];
                error(str)
            end
            %  1.2. GET PROPERTIES TO GET LESS EXECUTION TIMES 0.000012
            grid_XMIN   = this.grid_xmin;
            grid_YMIN   = this.grid_ymin;
            grid_DELTAX = this.grid_deltax;
            grid_DELTAY = this.grid_deltay;
            grid_ROWS   = this.grid_rows;
            grid_COLS   = this.grid_cols;
            grid_MAP    = this.grid_map;
            %   -> Get offset corresponding to the square of search
            INDEX_OFFSET_X = this.index_offset_square_xx;
            INDEX_OFFSET_Y = this.index_offset_square_yy;
            %   -> Get maximum squared distance
            dist2 = this.MAXIMUM_DISTANCE2;
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1 COMPUTE THE INDEXES[x,y] OF THE POINT IN THE GRID
            %   -> Get the indexes(1-based)  - 0.000004 seconds(OK)
            idx = ceil((xtest-grid_XMIN)/grid_DELTAX);
            idy = ceil((ytest-grid_YMIN)/grid_DELTAY);
            %   -> Check bottom left corner
            if(idx==0)
                idx = idx + 1;
            end
            if(idy==0)
                idy = idy + 1;
            end
            %  2.2 COMPUTE THE REPRESENTATIVE COORDINATES OF THE GRID 0.000006
            %   -> We choose the center of the grid as representative
            xtest_grid = (idx-1)*grid_DELTAX + grid_XMIN + grid_DELTAX/2;
            ytest_grid = (idy-1)*grid_DELTAY + grid_YMIN + grid_DELTAY/2;
            %  2.3 MAKE SURE THAT THE TEST POINT IS INSIDE THE GRID
            if(idx < 0 || idy < 0)
                str = ['occ_grid.search_closest_grid_cell:',...
                    'Negative index, location out of boundaries'];
                error(str)
            elseif(idx > grid_COLS || idy > grid_ROWS)
                str = ['occ_grid.search_closest_grid_cell:',...
                    'Positive index, location out of boundaries'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 3. CHECK THE GRID CELL OF THE TEST POINT 0.000005
            %-----------------------------------------------------------% 
            if(grid_MAP(idy,idx) == 1)
                % If the cell is OCCUPIED
                dist2 = 0;   
                xbar  = xtest_grid;
                ybar  = ytest_grid;
                return
            end
            
            %-----------------------------------------------------------%
            % 4. DEFINITION OF THE SQUARE OF SEARCH (Ok)0.000003
            %-----------------------------------------------------------%  
            %  4.1. SET 'botton-left' CORNER OF THE SQUARE
            idx_start = idx - INDEX_OFFSET_X;
            if(idx_start<1)
                idx_start = 1;
            end
            idy_start = idy - INDEX_OFFSET_Y;
            if(idy_start<1)
                idy_start = 1;
            end
            %   4.2 SET 'top-right' CORNER OF THE SQUARE
            idx_end = idx + INDEX_OFFSET_X;
            if(idx_end > grid_COLS)
                idx_end = grid_COLS;
            end
            idy_end = idy + INDEX_OFFSET_Y;
            if(idy_end > grid_ROWS)
                idy_end = grid_ROWS;
            end
            
            %-----------------------------------------------------------%
            % 5. COMPUTE THE MIN SQUARED DISTANCE "dist2" 
            %-----------------------------------------------------------%
            xbar = NaN;
            ybar = NaN;
            %   -> Run through each grid
            for mm = idy_start:idy_end      % rows
                for nn = idx_start:idx_end  % cols
                    % a. GET THE VALUE OF THE GRID (0=FREE OR 1=NOT)
                    flag = grid_MAP(mm, nn);
                    % b. COMPUTE THE MINIMUM DISTANCE TO AN OCCUPIED SPACE
                    if(flag==1)
                        % -> Coordinates of the center of the occupied grid
                        xx = (nn-1)*grid_DELTAX + grid_XMIN + grid_DELTAX/2;
                        yy = (mm-1)*grid_DELTAY + grid_YMIN + grid_DELTAY/2;
                        % -> Compute squared distance
                        tmp = (xx - xtest_grid)^2 + (yy - ytest_grid)^2;
                        % -> Save the minimum distance
                        if(tmp < dist2)
                            dist2 = tmp;
                            xbar = xx;
                            ybar = yy;
                        end
                    end
                end
            end
        end
        
        function DIST_LOCAL = compute_matrix_min_distance(this, ...
                GRID_LOCAL, GRID_IND)
            % occ_grid.compute_matrix_min_distance Matrix min distance
            %
            %   COMPUTE_MATRIX_MIN_DISTANCE(FLAG) Compute the matrix of
            %   "squared" minimum distances to an occupied grid cell
            %
            % OUTPUT
            %   The object with the matrix of minimum squared distances 
            %   from any grid cell to an occupied grid.
            %   This method also computes other values "xbar, ybar"
            %   Note that this function returns as minimum distance 
            %   'MAX_DIST=SQUARE_SEARCH_SIZE' if the occupied grid cell 
            %   is out of range               
            %
            % INPUT
            %   GRID_LOCAL: The portion of the grid map with the obstacles
            %               (Used in planner)
            %     GRID_IND: Indexes of the local grid
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. MAKE SURE THE SIZE OF THE SQUARE OF SEARCH IS DEFINED
            if(isempty(this.MAXIMUM_DISTANCE))
                str = ['occ_grid.compute_matrix_min_distance:',...
                    'MAXIMUM_DISTANCE is not defined'];
                error(str);
            end
            %  1.2. MAKE SURE index_offset_square_xx
            if(isempty(this.index_offset_square_xx))
                str = ['occ_grid.compute_matrix_min_distance:',...
                    'The parameter index_offset_square_xx is not defined'];
                error(str);
            end
            if(this.grid_TYPE==0)
                str = ['occ_grid.compute_matrix_min_distance:',...
                    'Does not work with IDEAL MAPS'];
                error(str);
            end
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1. GET PROPERTIES TO GET LESS EXECUTION TIMES 0.000012
            grid_XMIN       = this.grid_xmin;
            grid_YMIN       = this.grid_ymin;
            grid_DELTAX     = this.grid_deltax;
            grid_DELTAY     = this.grid_deltay;
            grid_ROWS       = this.grid_rows;
            grid_COLS       = this.grid_cols;
            grid_MAP        = this.grid_map;
            %GRID_XXX        = this.grid_XXX;
            %GRID_YYY        = this.grid_YYY;
            index_OFFSET_X  = this.index_offset_square_xx;
            index_OFFSET_Y  = this.index_offset_square_yy;
            MAX_DISTANCE2   = this.MAXIMUM_DISTANCE2;
            %  2.2. ALLOCATE MEMORY TO SAVE RESULTS
            min_dist2 = MAX_DISTANCE2*ones(grid_ROWS, grid_COLS);
            Xbar      = NaN*ones(grid_ROWS, grid_COLS);
            Ybar      = NaN*ones(grid_ROWS, grid_COLS);
            %  2.3. SET SPACE WHERE WE ARE GOINT TO COMPUTE THE DISTANCES
            if(nargin==1)
                % COMPUTE DISTANCES IN ALL THE GRID MAP
                ROW_INIT = 1;
                ROW_END  = grid_ROWS;
                COL_INIT = 1;
                COL_END  = grid_COLS;
            elseif(nargin==3)
                %   -> SET ROWS
                ROW_INIT = GRID_IND.ymin_idy;
                ROW_END  = GRID_IND.ymax_idy;
                if(ROW_INIT<1)
                    str = ['occ_grid.compute_matrix_min_distance:',...
                        'ROW_INIT is less than 1'];
                    error(str);
                end
                if(ROW_END > grid_ROWS)
                    str = ['occ_grid.compute_matrix_min_distance:',...
                        'ROW_END is bigger than grid_ROWS'];
                    error(str);
                end
                %   -> SET COLS
                COL_INIT = GRID_IND.xmin_idx;
                COL_END  = GRID_IND.xmax_idx;
                if(COL_INIT<1)
                    str = ['occ_grid.compute_matrix_min_distance:',...
                        'COL_INIT is less than 1'];
                    error(str);
                end
                if(COL_END > grid_COLS)
                    str = ['occ_grid.compute_matrix_min_distance:',...
                        'COL_END is bigger than grid_COLS'];
                    error(str);
                end
                
                %  SET THE LOCAL GRID AS PART OF THE FULL GRID
                grid_MAP(ROW_INIT:ROW_END, COL_INIT:COL_END) = GRID_LOCAL;
                
                % DISPLAY FULL MAP
                %figure('Name', 'grid map + local map')
                %imagesc(grid_MAP)
                %set(gca, 'Ydir', 'normal');
                %colorbar
            end
            
            %-----------------------------------------------------------%
            % 3. MAIN CODE - RUN THROUGH EVERY CELL OF THE GRID
            %-----------------------------------------------------------%
            for row = ROW_INIT:ROW_END          % idy
                for col = COL_INIT:COL_END      % idx
                    % A. COMPUTE THE REPRESENTATIVE COORDINATES OF THE GRID
                    %     -> We choose the center of the grid as representative
                    xtest_grid = (col-1)*grid_DELTAX + grid_XMIN + grid_DELTAX/2;
                    ytest_grid = (row-1)*grid_DELTAY + grid_YMIN + grid_DELTAY/2;
                    %xtest_grid = GRID_XXX(row, col);
                    %ytest_grid = GRID_YYY(row, col);
                    
                    % B. COMPUTE MINIMUM DISTANCE
                    if(grid_MAP(row, col) >= 1)     % If the cell is OCCUPIED
                                                    % for unknown spaces
                        dist2 = 0;
                        xbar  = xtest_grid;
                        ybar  = ytest_grid;
                    else % If the cell is FREE or UNKNOWN
                        %----------------------------------------------------%
                        % a. DEFINITION OF THE SQUARE OF SEARCH (Ok)0.000003
                        %----------------------------------------------------%
                        %  SET 'bottom-left' CORNER
                        idx_start = col - index_OFFSET_X;
                        if(idx_start < 1)
                            idx_start = 1;
                        end
                        idy_start = row - index_OFFSET_Y;
                        if(idy_start < 1)
                            idy_start = 1;
                        end
                        %  SET 'top-right' CORNER
                        idx_end = col + index_OFFSET_X;
                        if(idx_end > grid_COLS)
                            idx_end = grid_COLS;
                        end
                        idy_end = row + index_OFFSET_Y;
                        if(idy_end > grid_ROWS)
                            idy_end = grid_ROWS;
                        end
                        
                        %----------------------------------------------------%
                        % b. COMPUTE CLOSEST POINT
                        %----------------------------------------------------%
                        dist2 = MAX_DISTANCE2;
                        xbar = NaN;
                        ybar = NaN;
                        for mm = idy_start:idy_end      % rows
                            for nn = idx_start:idx_end  % cols
                                % a. Get the value of test grid map
                                flag_grid = grid_MAP(mm, nn);
                                % b. Compute the minimum to occupied space
                                if(flag_grid>=1)    % OCCUPIED (>= PLANNER)
                                    % -> Coordinates of the center of the grid
                                    xx = (nn-1)*grid_DELTAX + grid_XMIN +...
                                        grid_DELTAX/2;
                                    yy = (mm-1)*grid_DELTAY + grid_YMIN +...
                                        grid_DELTAY/2;
                                    %xx = GRID_XXX(mm, nn);
                                    %yy = GRID_YYY(mm, nn);
                                    % -> Compute squared distance
                                    tmp = (xx - xtest_grid)^2 + (yy - ytest_grid)^2;
                                    % -> Save the minimum distance(if inside
                                    %    a circle)
                                    if(tmp < dist2)     
                                        dist2 = tmp;
                                        xbar  = xx;
                                        ybar  = yy;
                                    end
                                end
                            end
                        end
                    end
                    
                    %----------------------------------------------------%
                    % c. SAVE RESULTS
                    %----------------------------------------------------%
                    min_dist2(row, col) = dist2;
                    Xbar(row, col)      = xbar;
                    Ybar(row, col)      = ybar;
                end
            end
            
            %-----------------------------------------------------------%
            % 4. SAVE PROPERTIES
            %-----------------------------------------------------------%
            if(nargin==1)
                % Matrix of minimum distances
                min_dist = sqrt(min_dist2);
                % Save data
                this.grid_map_min_distance  = min_dist;
                this.grid_map_min_distance2 = min_dist2;
                this.grid_min_distance_xbar = Xbar;
                this.grid_min_distance_ybar = Ybar;
            end
            
            %-----------------------------------------------------------%
            % 5. RETURN DISTANCES IN THE SELECTED SPACE (FOR PLANNER)
            %-----------------------------------------------------------%
            if(nargout==1)
                DIST_LOCAL = sqrt(min_dist2(ROW_INIT:ROW_END, COL_INIT:COL_END));
            end
        end
        
        function set_grid_map_min_distance(this, distances_file)
            % occ_grid.set_grid_map_min_distance Set the matrix of min
            %                                    distances of the grid map
            %
            %   SET_GRID_MAP_MIN_DISTANCE(GRID_MAP_MIN_DISTANCE) Sets the 
            %   property "grid_map_min_distance" and other info.
            %
            % INPUT
            %   GRID_MAP_MIN_DISTANCE: File that holds the min distances
            %   and its features
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. MAKE SURE THE ARGUMENT IS A STRING
            if(nargin==2 && ~ischar(distances_file))
                str = ['occ_grid.set_grid_map_min_distance:',...
                    'Argument must be a string'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. LOAD THE MATRIX OF MIN DISTANCES AND CHECK PROPERTIES
            %-----------------------------------------------------------%
            %  2.1 LOAD THE FILE THAT CONTAINS THE MATRIX
            if(exist(distances_file, 'file') == 2)
                load(distances_file, 'data')
            else
                fprintf(' occ_grid.set_grid_map_min_distance: Setting file: \n\t%s\n',...
                    distances_file)
                str = ['occ_grid.set_grid_map_min_distance:',...
                    'The file of distances does not exist'];
                error(str)
            end
            %  2.2. CHECK PROPERTIES
            %   -> Check dimensions
            if(size(this.grid_map) ~= size(data.min_distance) )
                str = ['occ_grid.set_grid_map_min_distance:',...
                    'Matrix dimensions does not match'];
                error(str)
            end
            %   -> Check xmin
            if(this.grid_xmin ~= data.grid_xmin)
                str = ['occ_grid.set_grid_map_min_distance:',...
                    'grid_xmin does not match'];
                error(str)
            end
            %   -> Check ymin
            if(this.grid_ymin ~= data.grid_ymin)
                str = ['occ_grid.set_grid_map_min_distance:',...
                    'grid_ymin does not match'];
                error(str)
            end
            %   -> Check deltax
            if(this.grid_deltax ~= data.deltax)
                str = ['occ_grid.set_grid_map_min_distance:',...
                    'grid_ymin does not match'];
                error(str)
            end
            %   -> Check type of map
            if(this.grid_TYPE ~= data.grid_TYPE)
                str = ['occ_grid.set_grid_map_min_distance:',...
                    'TYPE does not coincide'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 3. SET THE PROPERTIES OF THE MINIMUM DISTANCES
            %-----------------------------------------------------------%
            %  3.1. SET THE MINIMUM DISTANCES
            this.grid_map_min_distance  = data.min_distance;
            this.grid_map_min_distance2 = data.min_distance2;
            %  3.2. SET THE COORDINATES OF THE CLOSEST OCCUPIED GRID TO 
            %       ANY GRID CELL
            this.grid_min_distance_xbar = data.min_distance_xbar;
            this.grid_min_distance_ybar = data.min_distance_ybar;
            %  3.3. SET THE "SQUARE_SEARCH_SIZE" USED TO COMPUTE THE
            %       MINIMUM DISTANCES
            this.MAXIMUM_DISTANCE  = data.MAXIMUM_DISTANCE;
            this.MAXIMUM_DISTANCE2 = this.MAXIMUM_DISTANCE^2;
        end
               
        function save_matrix_min_distance(this, savefile)
            % occ_grid.save_matrix_min_distance
            %
            %   SAVE_MATRIX_MIN_DISTANCES() Save the matrix of minimum
            %   distances
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS CHECKING
            %-----------------------------------------------------------%
            if(nargin==1)
                grid_file = this.GRID_FILE;
                savefile  = [grid_file(1:end-5) '_min_distances.mat'];
            end
            
            %-----------------------------------------------------------%
            % 2. SET STRUCTURE TO SAVE DATA
            %-----------------------------------------------------------%
            data.min_distance       = this.grid_map_min_distance;
            data.min_distance2      = this.grid_map_min_distance2;
            data.min_distance_xbar  = this.grid_min_distance_xbar;
            data.min_distance_ybar  = this.grid_min_distance_ybar;
            data.MAXIMUM_DISTANCE   = this.MAXIMUM_DISTANCE;
            data.grid_xmin          = this.grid_xmin;
            data.grid_ymin          = this.grid_ymin;
            data.deltax             = this.grid_deltax;
            data.grid_TYPE          = this.grid_TYPE;
            
            %-----------------------------------------------------------%
            % 3. SAVE THE MATRIX
            %-----------------------------------------------------------%
            save(savefile, 'data')
        end
        %------------- END MINIMUM DISTANCES FUNCTIONS ---------------%
        
        
        %------------------ TO MANAGE THE ROBOT ----------------------%
        function set_robot(this, name_robot)
            % occ_grid.set_robot Set the the robot of "my_robot"
            %
            %   SET_ROBOT Sets the type of robot to be used by the user. 
            %   Also set the coordinates of the body of the robot, this 
            %   is used in the 'MAPPER', and in the 'PLANNER'.
            %
            % INPUT
            %   name_robot: 'R2D2-00', 'R2D2-01', etc
            %
            % NOTE
            %   - In the 'MAPPER' the robot's body coordinates are set by
            %     the method "set_occupancy_grid_map_dimensions"
            
            %-----------------------------------------------------------%
            % SET THE ROBOT
            %-----------------------------------------------------------%
            %  1.1. CREATE AN OBJECT "my_robot"
            this.my_robot = robot();
            %  1.2. SET PROPERTIES OF THE ROBOT AND ITS BODY COORDINATES
            switch(name_robot)
                case {'R2D2-00', 'R2D2-01'}
                    this.NAME_ROBOT = name_robot;
                    this.my_robot.set_robot(name_robot);
                    this.set_R2D2_body()
                case {'R2D2-R1', 'R2D2-R2'}
                    this.NAME_ROBOT = name_robot;
                    this.my_robot.set_robot(name_robot);
                    this.set_R2D2_RX_body()
                otherwise
                    error('occ_grid.set_robot: Incorrect name of robot')
            end
        end
        
        function display_gaussian(this,mu,P,color)
            % occ_grid.display_gaussian Display gaussian
            %
            %   DISPLAY_GAUSSIAN Display a Gaussian in the grid map
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 3)
                str = ['occ_grid.display_gaussian:',...
                    'Insufficient number of arguments.'];
                error(str);
            end
            %  1.2. SET DEFAULT COLOR
            if(nargin < 4)
                color = 'b';
            end
            %  1.3. SET PARAMETERS
            NN = 12;
            BB = 0.05;
            
            %-----------------------------------------------------------%
            % 2. COMPUTE THE POINTS THAT REPRESENT THE MEAN
            %-----------------------------------------------------------%            
            %  2.1. GET THE LOCATION
            xx = mu(1); 
            yy = mu(2);
            %  2.2. GET THE ORIENTATION
            theta = mu(3);
            if(theta > pi ||theta <= -pi)
                str = ['occ_grid.display_gaussian:',...
                    'The angle theta is not in the range.'];
                error(str);
            end     
            %  2.3. COMPUTE THE POINTS OF POSITION
            ang = linspace(0,2*pi,NN)';
            x_circle = xx*ones(NN,1) + BB*cos(ang);
            y_circle = yy*ones(NN,1) + BB*sin(ang);
            %  2.4. COMPUTE THE POINTS OF THE ORIENTATION
            xf = xx + BB*cos(theta);
            yf = yy + BB*sin(theta);
            linex = [xx  xf];
            liney = [yy  yf];
            
            %-----------------------------------------------------------%
            % 3. COMPUTE THE POINTS THAT REPRESENT THE COVARIANZE
            %-----------------------------------------------------------%  
            ellipse_points = sigma_ellipse(mu(1:2), P(1:2,1:2)+1e-8*eye(2), 2);
            
            %-----------------------------------------------------------%
            % 4. DRAW THE GAUSSIAN
            %-----------------------------------------------------------%
            %  4.1. DRAW THE MEAN
            set(this.h_mu(1),'xdata',x_circle,'ydata',y_circle,'Color',color);
            set(this.h_mu(2),'xdata',linex,'ydata',liney,'Color',color);
            %  4.2. DRAW THE COVARIANZE - ELLIPSE
            set(this.h_P,'xdata',ellipse_points(1,:),...
                'ydata',ellipse_points(2,:),'Color',color)
        end
        
        function display_robot(this,x,color)
            % occ_grid.display_robot Display robot
            %
            %   DISPLAY_ROBOT Display the robot in the 'x' pose
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['occ_grid.display_robot:',...
                    'Insufficient number of arguments.'];
                error(str);
            end
            %  1.2. SET DEFAULT COLOR
            if(nargin < 3)
                color = 'b';
            end
            %  1.3. SET PARAMETERS
            NN = 12;
            BB = 0.1;
            
            %-----------------------------------------------------------%
            % 2. GET THE POSE OF THE ROBOT
            %-----------------------------------------------------------%            
            %  2.1. GET THE LOCATION
            xx = x(1);
            yy = x(2);
            %  2.2. GET THE ORIENTATION
            theta = x(3);
            if(theta>pi ||theta<=-pi)
                str = ['occ_grid.display_robot:',...
                    'The angle theta is not in the range.'];
                error(str);
            end
            
            %-----------------------------------------------------------%
            % 3. COMPUTE REPRESENTATIVE POINTS OF THE ROBOT
            %-----------------------------------------------------------%
            %  3.1. COMPUTE THE POINTS OF THE CIRCLE
            ang = linspace(0,2*pi,NN)';
            x_circle = xx*ones(NN,1) + BB*cos(ang);     % x-components
            y_circle = yy*ones(NN,1) + BB*sin(ang);     % y_components
            %  3.2. COMPUTE THE POINTS OF THE ORIENTATION
            xf = xx + BB*cos(theta);
            yf = yy + BB*sin(theta);
            linex = [xx  xf];
            liney = [yy  yf];
            
            %-----------------------------------------------------------%
            % 4. DRAW THE NEW POSE USING THE HANDLES
            %-----------------------------------------------------------%
            set(this.h_robot(1),'xdata',x_circle,'ydata',y_circle,'Color',color);
            set(this.h_robot(2),'xdata',linex,'ydata',liney,'Color',color);
        end     
                  
        function get_robot_index_coordinates(this, x)
            % occ_grid.get_robot_index_coordinates
            %
            %   GET_ROBOT_INDEX_COORDINATES Get the indexes in the grid
            %   map that correspond to the robot's body
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK THE BODY OF THE ROBOT
            if(isempty(this.XX_robot))
                str = ['occ_grid.get_robot_index_coordinates:',...
                    'The body of the robot XX_robot is not defined'];
                error(str);
            end
            %  1.2. CHECK THE ANGLE OF THE POSE
            if(x(3)>pi || x(3)<=-pi)
                str = ['occ_grid.get_robot_index_coordinates:',...
                    'The body of the robot XX_robot is not defined'];
                error(str);
            end
            
            %---------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %---------------------------------------------------------%
            %  2.1. GET LOCAL COORDINATES OF THE ROBOT
            XX_ROBOT = this.XX_robot;
            YY_ROBOT = this.YY_robot;
            %  2.2. GET THE POSITION AND ORIENTATION 
            position    = x(1:2)';
            orientation = x(3);
            
            %---------------------------------------------------------%
            % 3. COMPUTE THE COORDINATES OF THE ROBOT'BODY IN ITS
            %    CURRENT POSE "x"
            %---------------------------------------------------------%
            %  3.1. COMPUTE COORDINATES OF THE ROBOT
            %   -> Convert these to polar coordinates
            [ang, radio] = cart2pol(XX_ROBOT, YY_ROBOT);
            %   -> Convert these back to cartesian coord. but with the 
            %      new orientation
            [x_car, y_car] = pol2cart(ang + orientation, radio);
            %   -> Set the new positions
            x_car = x_car + position(1);
            y_car = y_car + position(2);
            %  3.2. GET INDEXES OF THE ROBOT'S BODY IN THE GRID MATRIX 
            [IDX_CAR, IDY_CAR] = this.compute_index_from_location(...
                x_car, y_car);
            %  3.3. SAVE ROBOT'S BODY INDEXES
            this.IDX_robot = IDX_CAR;   % [Nx1]
            this.IDY_robot = IDY_CAR;   % [Nx1]
        end
        %-------------------------- END  -----------------------------%
    end
    
    %------------------------------------------------------------------%
    %  PRIVATE METHODS
    %------------------------------------------------------------------%
    methods(Access=private)
        function set_R2D2_body(this)
            % occ_grid.set_R2D2_body Set R2D2's robot's body
            %
            %   SET_R2D2_BODY Set the body coordinates of the R2D2 family
            %   of robots.
            %
            % NOTE
            %   - We have expanded some dimensions for planning and
            %     replanning considerations
            
            %--------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %--------------------------------------------------------%
            %  1.1. CHECK
            if(isempty(this.grid_deltax))
                error('occ_grid.set_R2D2_body: deltax is undefined')
            end
            %  1.2. GET PROPERTIES OF THE GRID
            DELTAX = this.grid_deltax;
            DELTAY = this.grid_deltay;
            %  1.3. GET DIMENSIONS OF THE ROBOT
            LENGTH = this.my_robot.DIMENSIONS.Length;
            WIDTH  = this.my_robot.DIMENSIONS.Width;
            EJE    = this.my_robot.DIMENSIONS.Eje;
            WW     = this.my_robot.DIMENSIONS.WW;
            
            %--------------------------------------------------------%
            % 2. COMPUTE COORDINATES OF THE ROBOT'S BODY WHEN ITS POSE
            %    IS [0;0;0]
            %--------------------------------------------------------%
            OFFSET = 0.04;
            %  2.1. SET THE COORDINATES OF THE CORNERS (local coord.)
            xmin_robot = -(LENGTH-EJE) - 0.01;
            xmax_robot = EJE           + 0.02;
            ymin_robot = -0.5*WIDTH    - OFFSET;
            ymax_robot =  0.5*WIDTH    + OFFSET;
            %  2.2. SET 'REPRESENTATIVE' COORDINATES
            xx = xmin_robot:DELTAX*0.8:xmax_robot;
            yy = ymin_robot:DELTAY*0.8:ymax_robot;
            %  2.3. GET MESGRID OF THE ROBOT'S BODY POINTS
            [XX, YY] = meshgrid(xx,yy);
            XX_body = reshape(XX,numel(XX),1);
            YY_body = reshape(YY,numel(YY),1);
            
            %--------------------------------------------------------%
            % 3. COMPUTE LOCAL COORDINATES OF THE ROBOT'S LEFT WHEEL
            %--------------------------------------------------------%
            OFFSET = 0.05;  % 0.10
            %  3.1. SET THE COORDINATES OF THE CORNERS
            xmin_robot = -EJE           - 0.01;
            xmax_robot =  EJE           + OFFSET;
            ymin_robot =  0.5*WIDTH     - OFFSET;
            ymax_robot =  0.5*WIDTH+WW  + OFFSET;
            %  3.2. SET 'REPRESENTATIVE' COORDINATES
            xx = xmin_robot:DELTAX*0.8:xmax_robot;
            yy = ymin_robot:DELTAY*0.8:ymax_robot;
            %  3.3. GET MESGRID OF THE ROBOT'S BODY
            [XX, YY] = meshgrid(xx,yy);
            XX_wheel_left = reshape(XX,numel(XX),1);
            YY_wheel_left = reshape(YY,numel(YY),1);
            
            %--------------------------------------------------------%
            % 4. COMPUTE LOCAL COORDINATES OF THE ROBOT'S RIGHT WHEEL
            %--------------------------------------------------------%
            %  4.1. SET THE COORDINATES OF THE CORNERS
            xmin_robot = -EJE             - 0.01;
            xmax_robot =  EJE             + OFFSET;
            ymin_robot = -(0.5*WIDTH+WW)  - OFFSET;
            ymax_robot = -0.5*WIDTH       + OFFSET;
            %  4.2. SET 'REPRESENTATIVE' COORDINATES
            xx = xmin_robot:DELTAX*0.8:xmax_robot;
            yy = ymin_robot:DELTAY*0.8:ymax_robot;
            %  4.3. GET MESGRID OF THE ROBOT'S BODY
            [XX, YY] = meshgrid(xx,yy);
            XX_wheel_right = reshape(XX,numel(XX),1);
            YY_wheel_right = reshape(YY,numel(YY),1);
            
            %--------------------------------------------------------%
            % 5. SAVE COORDINATES
            %--------------------------------------------------------%
            this.XX_robot = [XX_body;XX_wheel_left;XX_wheel_right];
            this.YY_robot = [YY_body;YY_wheel_left;YY_wheel_right];
        end
        
        function set_R2D2_RX_body(this)
            % occ_grid.set_R2D2_RX_body Set R2D2-RX's robot's body
            %
            %   SET_R2D2_RX_BODY Set the body coordinates of the R2D2-RX
            %   family of robots.
            
            %--------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %--------------------------------------------------------%
            %  1.1. CHECK
            if(isempty(this.grid_deltax))
                error('occ_grid.set_R2D2_RX_body: deltax is undefined')
            end
            %  1.2. GET PROPERTIES OF THE GRID
            DELTAX = this.grid_deltax;
            DELTAY = this.grid_deltay;
            %  1.3. GET DIMENSIONS OF THE ROBOT
            L1 = this.my_robot.DIMENSIONS.L1;
            L2 = this.my_robot.DIMENSIONS.L2;
            L3 = this.my_robot.DIMENSIONS.L3;
            L4 = this.my_robot.DIMENSIONS.L4;
            
            %--------------------------------------------------------%
            % 2. COMPUTE COORDINATES OF THE ROBOT'S BODY 1 WHEN ITS
            %    POSE IS [0;0;0]
            %--------------------------------------------------------%
            OFFSET = 0.04;
            %  2.1. SET THE COORDINATES OF THE CORNERS (local coord.)
            xmin_robot = -L2/2 - OFFSET;
            xmax_robot =  L2/2 + OFFSET;
            ymin_robot = -L1/2 - OFFSET;
            ymax_robot =  L1/2 + OFFSET;
            %  2.2. SET 'REPRESENTATIVE' COORDINATES
            xx = xmin_robot:DELTAX*0.8:xmax_robot;
            yy = ymin_robot:DELTAY*0.8:ymax_robot;
            %  2.3. GET MESGRID OF THE ROBOT'S BODY POINTS
            [XX, YY] = meshgrid(xx,yy);
            XX_body1 = reshape(XX,numel(XX),1);
            YY_body1 = reshape(YY,numel(YY),1);
            
            %--------------------------------------------------------%
            % 3. COMPUTE COORDINATES OF THE ROBOT'S BODY 2 WHEN ITS
            %    POSE IS [0;0;0]
            %--------------------------------------------------------%
            OFFSET = 0.02;
            %  2.1. SET THE COORDINATES OF THE CORNERS (local coord.)
            xmin_robot = -L4-L2/2 - OFFSET;
            xmax_robot = -L2/2 + OFFSET;
            ymin_robot = -L3/2 - OFFSET;
            ymax_robot =  L3/2 + OFFSET;
            %  2.2. SET 'REPRESENTATIVE' COORDINATES
            xx = xmin_robot:DELTAX*0.7:xmax_robot;
            yy = ymin_robot:DELTAY*0.7:ymax_robot;
            %  2.3. GET MESGRID OF THE ROBOT'S BODY POINTS
            [XX, YY] = meshgrid(xx,yy);
            XX_body2 = reshape(XX,numel(XX),1);
            YY_body2 = reshape(YY,numel(YY),1);
            
            %--------------------------------------------------------%
            % 5. SAVE COORDINATES
            %--------------------------------------------------------%
            this.XX_robot = [XX_body1;XX_body2];
            this.YY_robot = [YY_body1;YY_body2];
        end
    end
    
    %------------------------------------------------------------------%
    %  STATIC METHODS
    %------------------------------------------------------------------%
    methods(Static)
        function credits()
            % occ_grid.credits Show credits
            
            %-----------------------------------------------------------%
            % 1. MESSAGES
            %-----------------------------------------------------------%
            disp('-------------------------------------------')
            disp('---  ABOUT THE "R2D2" GRID MAP LIBRARY  ---')
            disp('-------------------------------------------')
            disp('  Author: Ivan A. Calle Flores       ')
            disp('  e-mail: ivan.calle.flores@gmail.com')
        end
    end 
end
