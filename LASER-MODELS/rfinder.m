% RFINDER
%
%   This class implements the methods and properties to handle
%   range-finder probabilistic likelihood model.
classdef rfinder < handle
    %------------------------------------------------------------------%
    %  DECLARE THIS PROPERTIES SUCH THAT
    %   - Can only be accesed inside the class
    %   - Can be read from anywhere
    %------------------------------------------------------------------%
    properties (SetAccess=protected, GetAccess=public)
        %------------------------------------------------------------%
        % PARAMETERS OF THE LIKELIHOOD FIELD
        %------------------------------------------------------------%
        my_occ_grid     % Object of class "occ_grid"
        
        %------------------------------------------------------------%
        % PARAMETERS OF THE LIKELIHOOD FIELD
        %------------------------------------------------------------%
        %  MIXING WEIGHTS. EQ(6.34)
        zhit
        zrand
        zmax
        %  STANDARD DEVIATION OF THE GAUSSIAN. EQ(6.33)
        sigma_hit
        %  FLAGS OF OPERATION
        IS_READY = 0        % READINESS FLAG
        MODE_DISTANCES      % 'KNOWN', 'UNKNOWN' 
        
        %------------------------------------------------------------%
        % FEATURES OF THE LASER RANGE FINDER
        %------------------------------------------------------------%
        %  FEATURES OF THE SELECTED BEAMS FOR LOCALIZATION
        theta_laser_LIKELIHOOD;
        K_LIKELIHOOD            % Number of selected beams
        max_range               % Maximum range
        %  POSE OF THE SENSOR IN THE ROBOT'S FRAME
        xk              % Axis in the direction of motion
        yk
        thk             % Orientation with respect to axis "xk"
    end
    
    %------------------------------------------------------------------%
    % PUBLIC METHODS
    %------------------------------------------------------------------%
    methods
        %---------------------- CONSTRUCTOR --------------------------%
        function this = rfinder()
            % rfinder.rfinder  Constructor function
            %
            %   RFINDER() returns a valid "rfinder" object.
            
            this.my_occ_grid = occ_grid();
        end
        
        function set_occ_grid(this, grid)
            % rfinder.set_occ_grid Sets an occ_grid object
            
            %----------------------------------------------------------%
            % 1. SET GRID_MAP AND MINIMUM DISTANCES FLAG
            %----------------------------------------------------------%
            this.my_occ_grid = grid;
            if(isempty(grid.grid_map_min_distance))
                str = ['rfinder.set_occ_grid:',...
                    'The minimum distances are not defined'];
                disp(str)
                this.MODE_DISTANCES = 'UNKNOWN';
            else
                str = ['rfinder.set_occ_grid:',...
                    'The minimum distances are defined'];
                disp(str)
                this.MODE_DISTANCES = 'KNOWN';
            end
        end
        
        function set_distances_flag(this, flag)
            % rfinder.set_distances_flag
            
            %-----------------------------------------------------------%
            % SET FLAG
            %-----------------------------------------------------------%
            switch(flag)
                case {'KNOWN', 'UNKNOWN'}
                    this.MODE_DISTANCES = flag;
                otherwise
                    error('rfinder.set_distances_flag: Undefined flag')
            end
        end
        
        function set_robot(this, name_robot)
            this.my_occ_grid.set_robot(name_robot);
        end
        %-------------------- END CONSTRUCTOR ------------------------%
        
        
        %---------------- RANGE FINDER BASIC FUNCTIONS ---------------%
        function set_max_range(this, max_range)
            % rfinder.set_max_range Set maximum range range of the laser
            %                        measurements
            %
            %   SET_MAX_RANGE(MAX_RANGE) Set the maximum range of the laser
            %   range finder
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['rfinder.set_max_range'...
                    'You must pass the range'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. SET THE PROPERTY
            %-----------------------------------------------------------%
            this.max_range = max_range;
        end
        
        function set_theta_laser_LIKELIHOOD(this, angles)
            % rfinder.set_theta_LIKELIHOOD Set angles of the likelihood 
            %                               model
            %
            %   SET_THETA_LASER_LIKELIHOOD(ANGLES) Set the angles of the
            %   laser's beams (in the laser frame) to be used to compute
            %   p(zx) using "likelihood_field" algorithm
            %
            % INPUTS
            %   angles: Laser beam's angles in the laser frame
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['rfinder.set_theta_LIKELIHOOD:', ...
                    'You must pass the angles of the laser beams'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. SET PROPERTIES
            %-----------------------------------------------------------%
            %  2.1. SET THE NUMBER OF BEAMS FOR LIKELIHOOD
            this.K_LIKELIHOOD = length(angles);
            %  2.2. SET ANGLES
            this.theta_laser_LIKELIHOOD = angles;
        end
        
        function set_laser_local_pose(this, local_pose)
            % rfinder.set_laser_local_pose
            %
            %   SET_LASER_LOCAL_POSE Set the pose of the sensor in the 
            %   robot's fixed frame
            %
            % INPUT
            %   local_pose: Pose of the laser in the robot's local frame
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK DIMENSION OF THE LOCAL POSE
            if(length(local_pose)~=3)
                str = ['rfinder.set_laser_local_pose:', ...
                    'Local pose must be of size 3'];
                error(str)
            end
            %  1.2. CHECK THE LOCAL ORIENTATION
            local_pose(3) = pi_to_pi(local_pose(3));
            
            %-----------------------------------------------------------%
            % 2. SET LOCAL POSE
            %-----------------------------------------------------------%
            this.xk  = local_pose(1);
            this.yk  = local_pose(2);
            this.thk = local_pose(3);
        end
        
        function local_pose = get_laser_local_pose(this)
            % laser.get_laser_local_pose
            %
            %   GET_LASER_LOCAL_POSE Get the pose of the sensor in the 
            %   robot's fixed frame      
            
            local_pose = [this.xk; this.yk; this.thk];
        end
        
        function global_pose = get_laser_global_pose(this, x)
            % rfinder.get_laser_global_pose
            %
            %   GET_LASER_GLOBAL_POSE Get the global pose of the laser
            %
            % OUTPUT
            %   global_pose: Global pose of the laser sensor
            %
            % INPUT
            %   x: Pose of the robot
            
            %-----------------------------------------------------------%
            %  1. GET PROPERTIES OF THE LASER
            %-----------------------------------------------------------%
            XK  = this.xk;
            YK  = this.yk;
            THK = this.thk;
            
            %-----------------------------------------------------------%
            % 2. COMPUTE LASER GLOBAL POSE
            %-----------------------------------------------------------%
            %  2.1. COMPUTE POSE
            theta = x(3);
            xx = x(1) + XK*cos(theta) - YK*sin(theta);
            yy = x(2) + XK*sin(theta) + YK*cos(theta);
            tt = x(3) + THK;
            %  2.2. SET OUTPUT
            global_pose = [xx; yy; tt];
        end
        %---------------- END FINDER BASIC FUNCTIONS -----------------%
        
        
        %-------------- LIKELIHOOD FIELD RANGE MODEL -----------------%
        function set_likelihood_field_parameters(this, zhit, zrand,...
                zmax, sigma_hit)
            % rfinder.set_likelihood_field_parameters Set parameters of the
            %                                         likelihood model
            %
            %   SET_LIKELIHOOD_FIELD_PARAMETERS(ZHIT,ZRAND,ZMAX,SIGMA_HIT)
            %   Set the instrinsic parameters of the likelihood field
            %   range model
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 5)
                str= ['rfinder.set_likelihood_field_parameters:',...
                    'Insuficient number of parameters'];
                error(str);
            end
            
            %-----------------------------------------------------------%
            % 2. MAIN CODE
            %-----------------------------------------------------------%
            %  2.1. MIXING WEIGHTS
            this.zhit  = zhit;
            this.zrand = zrand;
            this.zmax  = zmax;
            %  2.2. STD OF THE DISTANCE
            this.sigma_hit = sigma_hit;
        end
        
        function [Q, end_points, Keff] = ...
                likelihood_field_range_finder_model(this, z, X, DEBUG)
            % rfinder.likelihood_field_range_finder_model Likelihood field
            %                                              model
            %
            %   LIKELIHOOD_FIELD_RANGE_FINDER_MODEL Computes the likelihood
            %   of a range finder scan for a given pose. Table 6.3. Page 172
            %
            % OUTPUTS
            %           Q: The likelihood of the scan "p(z|x,m)" for every
            %              test pose
            %  end_points: The end points of the this scan in the global 
            %              coordinates
            %        keff: Number of non-maximum range measurements
            %
            % INPUTS
            %         z: Laser scan
            %         X: Test poses (Each row is a pose)
            %  DEBUG: Flag to show debug information
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 3)
                str = ['rfinder.LIKELIHOOD_FIELD_RANGE_FINDER_MODEL:',...
                    'Insuficient number of parameters'];
                error(str)
            end
            %  1.2. IF "X" IS A COLUMN, CONVERT IT TO A COLUMN
            if((size(X,2)==1) && length(X)==3)
                X = X(:)';
            end
            %  1.3. CHECK FOR DIMENSION OF THE POSE
            if(size(X,2)~=3)
                str = ['rfinder.LIKELIHOOD_FIELD_RANGE_FINDER_MODEL:',...
                    'X must have 3 columns'];
                error(str)
            end
            %  1.4. SET DEFAULT VALUE FOR THE DEBUG FLAG
            if(nargin < 4)
                DEBUG = 0;
            end
            
            %-----------------------------------------------------------%
            % 2. COMPUTE LIKELIHOOD
            %-----------------------------------------------------------%
            %  2.1. CHECK THE MAIN ATRIBUTES
            if(this.IS_READY == 0)
                this.check_readiness();
            end
            %  2.2. COMPUTE LIKELIHOOD
            if(this.my_occ_grid.grid_TYPE == 0)
                [Q, end_points, Keff] = ...
                    this.likelihood_field_range_finder_model_IDEAL(z, X, DEBUG);
            else
                [Q, end_points, Keff] = ...
                    this.likelihood_field_range_finder_model_REAL(z, X);
            end
        end
        
        function check_readiness(this)
            % rfinder.check_readiness
            %
            %   CHECK_READINESS_IDEAL Check if the rfinder is ready to apply
            %   the "likelihood_field_range_finder_model_IDEAL"
            
            %-----------------------------------------------------------%
            % 1. CHECK PROPERTIES OF THE CLASS
            %-----------------------------------------------------------%
            %  1.1. MAKE SURE THE GRID MAP IS DEFINED
            if(isempty(this.my_occ_grid))
                str = ['rfinder.check_readiness:',...
                    'The grid map is not defined'];
                error(str)
            end
            %  1.2. MAKE SURE THE ANGLES OF THE "SELECTED" LASER BEAM'S 
            %       ARE DEFINED
            if(isempty(this.theta_laser_LIKELIHOOD))
                str = ['rfinder.check_readiness:',...
                    'The angles of the "SELECTED" laser beams are not defined'];
                error(str)
            end
            %  1.3. MAKE SURE THE PARAMETERS OF THE ALGORITH ARE DEFINED
            if(isempty(this.zhit) || isempty(this.zrand) || ...
                    isempty(this.zmax) || isempty(this.sigma_hit) )
                str = ['rfinder.check_readiness:',...
                    'Intrinsic parameters not defined'];
                error(str)
            end
            %  1.4. CHECK THAT THE LASER LOCAL POSE IS DEFINED
            if(isempty(this.xk))
                str = ['rfinder.check_readiness:',...
                    'The laser pose [xk,yk,thk] is not defined'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. SET_FLAG
            %-----------------------------------------------------------%
            this.IS_READY = 1;
        end
        %------------ END LIKELIHOOD FIELD RANGE MODEL ---------------%
        
        
        %---------- GRADIENT LIKELIHOOD FIELD RANGE MODEL ------------%
        function [dx, dy, dt, LOG] = ...
                gradient_likelihood_field(this, z, x, DEBUG)
            % occ_grid.gradient_likelihood_field Gradient of p(z|x)
            %
            %   GRADIENT_LIKELIHOOD_FIELD Computes the gradient of the
            %   likelihood model of a laser range finder
            %
            % OUTPUTS
            %    dx: Derivative of "p(z|x,m)" with respect to 'x'
            %    dy: Derivative of "p(z|x,m)" with respect to 'y'
            %    dt: Derivative of "p(z|x,m)" with respect to 'theta'
            %   LOG: Returns 'log p(z|x,m)'
            %
            % INPUTS
            %      z: Laser scan
            %      x: Test pose 
            %  DEBUG: Flag to show debug information
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 3)
                str = ['rfinder.gradient_likelihood_field:',...
                    'Insuficient number of parameters'];
                error(str)
            end
            %  1.2. CHECK FOR DIMENSION OF THE POSE
            if(length(x)~=3)
                str = ['rfinder.gradient_likelihood_field:',...
                    'The pose must be of length 3'];
                error(str)
            end
            %  1.3. SET DEFAULT VALUE FOR THE DEBUG FLAG
            if(nargin < 4)
                DEBUG = 0;
            end
            
            %-----------------------------------------------------------%
            % 2. COMPUTE GRADIENT LIKELIHOOD FIELD
            %-----------------------------------------------------------%
            %  2.1. CHECK THE MAIN ATRIBUTES
            if(this.IS_READY == 0)
                this.check_readiness();
            end
            %  2.2. COMPUTE GRADIENT
            switch(this.my_occ_grid.grid_TYPE)
                case 0 % Ideal grid maps
                    [dx, dy, dt, LOG] = ...
                        this.gradient_likelihood_field_IDEAL(z, x, DEBUG);
                case 1 % Real maps
                    [dx, dy, dt, LOG] = ...
                        this.gradient_likelihood_field_REAL(z, x, DEBUG);
                otherwise
                    str = ['rfinder.gradient_likelihood_field:',...
                        'undefined grid_TYPE'];
                    error(str);
            end
        end
        
        function [XNEW, FLAGS, STEPS, Q] = likelihood_gradient_search(this,...
                s, X)
            % rfinder.likelihood_gradient_search Gradient search
            %
            %   LIKELIHOOD_GRADIENT_SEARCH Finds the optimal pose that
            %   results of the maximization the log of 'p(z|x)' using the
            %   gradient search algorithm
            %
            % OUTPUTS
            %   xnew: Optimized pose
            %   FLAG: 1:Succesful search / 0:Otherwise
            %
            % INPUTS
            %   s: Laser measurements
            %   X: Initial search poses. (Each row is a pose)
            
            %-----------------------------------------------------------%
            % 1. CHECK PROPERTIES OF THE CLASS
            %-----------------------------------------------------------%          
            %  1.1. CHECK THE MAIN ATRIBUTES
            if(this.IS_READY == 0)
                this.check_readiness();
            end
            %  1.2. IF "X" IS A COLUMN, CONVERT IT TO A ROW
            IS_COLUMN = 0;
            if((size(X,2)==1) && length(X)==3)
                IS_COLUMN = 1;
                X = X(:)';
            end
            %  1.3. CHECK "X"
            if(size(X,2)~=3)
                str = ['rfinder.likelihood_gradient_search:',...
                    'X must have 3 columns'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. COMPUTE GRADIENT LIKELIHOOD FIELD
            %-----------------------------------------------------------%
            %  2.1. CHECK THE MAIN ATRIBUTES
            if(this.IS_READY == 0)
                this.check_readiness();
            end
            %  2.2. COMPUTE GRADIENT
            switch(this.my_occ_grid.grid_TYPE)
                case 0 % Ideal grid maps
                    DEBUG = 0;
                    [XNEW, FLAGS, STEPS, Q] = ...
                        this.likelihood_gradient_search_IDEAL(s, X, DEBUG);
                case 1 % Real maps
                    [XNEW, FLAGS, STEPS, Q] = ...
                        this.likelihood_gradient_search_REAL(s, X);
                otherwise
                    str = ['rfinder.likelihood_gradient_search:',...
                        'undefined grid_TYPE'];
                    error(str);
            end
            %  2.3. IF "X" IS A COLUMN, THE OUTPUT IS A COLUMN
            if(IS_COLUMN)
                XNEW = XNEW';
            end
        end
        %-------- END GRADIENT LIKELIHOOD FIELD RANGE MODEL ----------%
    end
    
    %------------------------------------------------------------------%
    %  PRIVATE METHODS
    %------------------------------------------------------------------%
    methods(Access=private)
        %---------------- LIKELIHOOD FIELD RANGE MODEL ----------------%
        function [Q, end_points, Keff] = ...
                likelihood_field_range_finder_model_IDEAL(this, z, X, DEBUG)
            % rfinder.likelihood_field_range_finder_model_IDEAL
            %
            %   LIKELIHOOD_FIELD_RANGE_FINDER_IDEAL Computes the likelihood
            %   of a range finder scan for ideal maps
            
            %-----------------------------------------------------------%
            % 1. GET PROPERTIES OF THE CLASS (To speed up)
            %-----------------------------------------------------------
            %  1.1. OF THE LASER
            MAX_RANGE              = this.max_range;
            KK_LIKELIHOOD          = this.K_LIKELIHOOD;
            THETA_LASER_LIKELIHOOD = this.theta_laser_LIKELIHOOD;
            xxk                    = this.xk;
            yyk                    = this.yk;
            tthk                   = this.thk;
            %  1.2. OF THE GRID
            my_grid               = this.my_occ_grid;
            GRID_XMIN             = my_grid.grid_xmin;
            GRID_YMIN             = my_grid.grid_ymin;
            GRID_DELTAX           = my_grid.grid_deltax;
            GRID_DELTAY           = my_grid.grid_deltay;
            GRID_COLS             = my_grid.grid_cols;
            GRID_ROWS             = my_grid.grid_rows;
            GRID_MAP_MIN_DISTANCE = my_grid.grid_map_min_distance;
            %  1.3. LIKELIHOOD MODEL
            ZHIT      = this.zhit;
            ZRAND     = this.zrand;
            ZMAX      = this.zmax;
            SIGMA_HIT = this.sigma_hit;
            %  1.4. CHECK MIN DISTANCES   
            if(isempty(GRID_MAP_MIN_DISTANCE) )
                str = ['rfinder.likelihood_field_range_finder_model_IDEAL:',...
                    'The minimum distances are not defined'];
                error(str)
            end
            
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1. GET NUMBER OF TEST POSES
            NN = size(X,1);
            %  2.2. OUTPUTS CONFIGURATION
            Q = zeros(NN,1);
            %  2.3. MATRIX TO SAVE THE END POINTS              
            end_points = NaN*ones(KK_LIKELIHOOD,2);
            
            %-----------------------------------------------------------%
            % 3. COMPUTE PROBABILITIES
            %-----------------------------------------------------------%
            for mm=1:NN
                %----------------------------------------------------%
                % 3.1 GET A POSE
                %----------------------------------------------------%
                xtest = X(mm,:);
                xx    = xtest(1);
                yy    = xtest(2);
                theta = xtest(3);
                
                %----------------------------------------------------%
                % 3.2 MAIN LOOP
                %----------------------------------------------------%
                q = 1;
                Keff = 0;
                for i=1:KK_LIKELIHOOD
                    % A. CHECK MAXIMUM RANGE MEASUREMENTS
                    if(z(i) == MAX_RANGE)
                        if(DEBUG)
                            fprintf('   - %d measurement: Maximum range\n', i)
                            fprintf('      Do not take this mmeasurement\n')
                        end
                        continue
                    end
                    Keff = Keff + 1;
                    
                    % B. COMPUTE THE END POINT OF THE MEASUREMENTS.
                    xz = xx + xxk*cos(theta) - yyk*sin(theta) +...
                            z(i)*cos(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                    yz = yy + xxk*sin(theta) + yyk*cos(theta) + ...
                            z(i)*sin(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                    %   -> Show messages for debug
                    if(DEBUG)
                        fprintf('   - %d measurement: [xz:%2.4f, yz:%2.4f]\n', i, xz, yz)
                    end
                    %   -> Save end points
                    end_points(i,:) = [xz, yz];
                    
                    % C. COMPUTE THE INDEXES[x,y] THAT CORRESPOND TO THE
                    %      LASER END POINTS IN THE GRID
                    %   -> Get the indexes(1-based)
                    idx = ceil((xz-GRID_XMIN )/GRID_DELTAX);
                    idy = ceil((yz-GRID_YMIN )/GRID_DELTAY);
                    %   -> Check for bottom left corner
                    if(idx==0)
                        idx = idx + 1;
                    end
                    if(idy==0)
                        idy = idy + 1;
                    end
                    %   -> Show messages for debug
                    if(DEBUG)
                        fprintf('      Grid index of end point[row, col] = [%d, %d] \n', idy, idx)
                    end
                    
                    % D. COMPUTE THE LIKELIHOOD OF THE CURRENT SCAN
                    if((idx<1 || idx>GRID_COLS) || (idy<1 || idy>GRID_ROWS))
                        % IF THE END POINT FALLS BEYOND THE LIMITS OF THE GRID MAP
                        %   -> Set default constant value
                        qq = 0.15;
                        %   -> Show messages for debug
                        if(DEBUG)
                            fprintf('     This grid position for the end point is beyond limits\n')
                        end
                    else
                        % IF THE END FALLS WITHIN THE LIMITS OF THE GRID MAP
                        %   -> Get the minimum "PRE-COMPUTED" distance to an 
                        %      occupied space(if grid is free). Also distance
                        %      to an free space(if grid is occupied)
                        dist = GRID_MAP_MIN_DISTANCE(idy, idx);
                        if(DEBUG)
                            fprintf('      Minimum distance = %2.4f \n', dist)
                        end
                        %   -> Compute the likelihood
                        qq = ZHIT*gauss_1d(0, SIGMA_HIT^2, dist) +...
                            ZRAND/ZMAX;
                    end
                    %   -> Show messages for debug
                    if(DEBUG)  
                        fprintf('      Likelihood = %2.4f \n', qq)
                    end
                    
                    % E. ACCUMULATE THE LIKELIHOOD ASSUMING CONDITIONAL 
                    %      INDEPENDENCE
                    q = q*qq;                   
                end
                
                %----------------------------------------------------%
                % 3.3. SAVE RESULTS
                %----------------------------------------------------%
                Q(mm) = q;
            end
        end
        
        function [Q, end_points, Keff] = ...
                likelihood_field_range_finder_model_REAL(this, z, X)
            % rfinder.likelihood_field_range_finder_model_REAL
            %
            %   LIKELIHOOD_FIELD_RANGE_FINDER_REAL computes the likelihood 
            %   of a range finder scan for real maps
            %
            % NOTES: 
            %   - Big lesson here: it is more efficient to use the
            %     function "gauss_1d" outside of the class
                   
            %-----------------------------------------------------------%
            % 1. GET PROPERTIES OF THE CLASS
            %-----------------------------------------------------------%
            %  1.1. OF THE LASER
            MAX_RANGE              = this.max_range;
            KK_LIKELIHOOD          = this.K_LIKELIHOOD;
            THETA_LASER_LIKELIHOOD = this.theta_laser_LIKELIHOOD;
            xxk                    = this.xk;
            yyk                    = this.yk;
            tthk                   = this.thk;
            %  1.2. OF THE GRID MAP
            my_grid               = this.my_occ_grid;
            GRID_MAP              = my_grid.grid_map;
            GRID_XMIN             = my_grid.grid_xmin;
            GRID_YMIN             = my_grid.grid_ymin;
            GRID_DELTAX           = my_grid.grid_deltax;
            GRID_DELTAY           = my_grid.grid_deltay;
            GRID_COLS             = my_grid.grid_cols;
            GRID_ROWS             = my_grid.grid_rows;
            GRID_MAP_MIN_DISTANCE = my_grid.grid_map_min_distance;
            %  1.3. OF THE LIKELIHOOD MODEL
            ZHIT      = this.zhit;
            ZRAND     = this.zrand;
            ZMAX      = this.zmax;
            SIGMA_HIT = this.sigma_hit;
            %  1.4. SET MINIMUM DISTANCE THRESHOLD
            if(strcmp(this.MODE_DISTANCES, 'UNKNOWN'))
                my_grid.set_maximum_distance(0.4);
            end
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1. GET NUMBER OF TEST POSES
            NN = size(X,1);
            %  2.2. OUTPUTS CONFIGURATION
            Q = zeros(NN,1);
            %  2.3. PROBABILITY OF DISTANCE 0(IDEAL MEASUREMENT)
            q_MAX = ZHIT*gauss_1d(0, SIGMA_HIT^2, 0) + ZRAND/ZMAX;
            %  2.4. MATRIX TO SAVE THE END POINTS              
            end_points = NaN*ones(KK_LIKELIHOOD,2);
            
            %-----------------------------------------------------------%
            % 3. COMPUTE PROBABILITIES
            %-----------------------------------------------------------%
            for mm=1:NN
                %----------------------------------------------------%
                % 3.1 GET A POSE
                %----------------------------------------------------%
                xtest = X(mm,:);
                xx    = xtest(1);
                yy    = xtest(2);
                theta = xtest(3);
                
                %-----------------------------------------------------%
                % 3.2. MAIN LOOP
                %-----------------------------------------------------%
                q = 1;
                Keff = 0;
                for i=1:KK_LIKELIHOOD
                    % A. CHECK MAXIMUM RANGE MEASUREMENTS
                    if(z(i) == MAX_RANGE)
                        %fprintf('   - %d measurement: Maximum range\n', i)
                        %fprintf('      Do not take this mmeasurement\n')
                        continue
                    end
                    Keff = Keff + 1;
                    
                    % B. COMPUTE THE END POINT OF THE MEASUREMENTS
                    xz = xx + xxk*cos(theta) - yyk*sin(theta) +...
                            z(i)*cos(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                    yz = yy + xxk*sin(theta) + yyk*cos(theta) + ...
                            z(i)*sin(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                    % fprintf('   - %d measurement: [xz:%2.4f, yz:%2.4f]\n', i, xz, yz)
                    %   -> Save end points
                    end_points(i,:) = [xz, yz];
                    
                    % C. COMPUTE THE INDEXES[x,y] THAT CORRESPOND TO THE
                    %      LASER END POINTS IN THE GRID
                    %   -> Get the indexes(1-based)
                    idx = ceil((xz-GRID_XMIN)/GRID_DELTAX);
                    idy = ceil((yz-GRID_YMIN)/GRID_DELTAY);
                    %   -> Check for bottom left corner
                    if(idx==0)
                        idx = idx + 1;
                    end
                    if(idy==0)
                        idy = idy + 1;
                    end
                    %fprintf('     Grid index of end point[row, col] = [%d, %d] \n', idy, idx)
                    
                    % D. COMPUTE THE LIKELIHOOD OF THE CURRENT SCAN
                    if( (idx<1 || idx>GRID_COLS) || (idy<1 || idy>GRID_ROWS))
                        % IF THE END POINT FALLS BEYOND THE LIMITS OF THE GRID MAP
                        qq = 0.15;
                        %fprintf('     This grid position for the end point is beyond limits\n')
                    else
                        % IF THE END-POINT FALLS WITHIN THE LIMITS OF THE GRID MAP
                        if(GRID_MAP(idy, idx) == 1)     % OCCUPIED
                            qq = q_MAX;
                            % fprintf('     End point falls is an OCCUPIED grid\n')
                        else    % FREE OR UNKNOWN
                            % Get the minimum "PRE-COMPUTED" distance to  
                            % an occupied space
                            switch(this.MODE_DISTANCES)
                                case 'KNOWN'
                                    dist = GRID_MAP_MIN_DISTANCE(idy, idx);
                                case 'UNKNOWN'
                                    dist = sqrt(my_grid.search_closest_grid_cell(xz, yz));
                            end
                            % fprintf('     End point falls is a FREE OR UNKNOWN grid\n')
                            % fprintf('     Minimum distance = %2.4f \n', dist)
                            % Compute the likelihood
                            qq = ZHIT*gauss_1d(0, SIGMA_HIT^2, dist) + ZRAND/ZMAX;
                        end
                    end
                    %fprintf('     Likelihood = %2.4f \n', qq)
                    
                    % e. ACCUMULATE THE LIKELIHOOD ASSUMING CONDITIONAL 
                    %    INDEPENDENCE
                    q = q*qq;
                end
                
                %----------------------------------------------------%
                % 3.3. SAVE RESULTS
                %----------------------------------------------------%
                Q(mm) = q;
            end
        end
        %--------------------------------------------------------------%
        
        %------------ GRADIENT LIKELIHOOD FIELD RANGE MODEL -----------% 
        function [dx, dy, dt, LOG] = ...
                gradient_likelihood_field_IDEAL(this, z, x, DEBUG)
            % occ_grid.gradient_likelihood_field_IDEAL Gradient of p(z|x)
            %
            %   GRADIENT_LIKELIHOOD_FIELD_IDEAL Computes the gradient of
            %   the likelihood model of a laser range finder using IDEAL
            %   grid maps
                        
            %-----------------------------------------------------------%
            % 1. GET PROPERTIES(To speed up)
            %-----------------------------------------------------------%
            %  1.1. OF THE LASER
            MAX_RANGE              = this.max_range;
            KK_LIKELIHOOD          = this.K_LIKELIHOOD;
            THETA_LASER_LIKELIHOOD = this.theta_laser_LIKELIHOOD;
            xxk                    = this.xk;
            yyk                    = this.yk;
            tthk                   = this.thk;
            %  1.2. OF THE GRID
            my_grid               = this.my_occ_grid;
            GRID_XMIN             = my_grid.grid_xmin;
            GRID_YMIN             = my_grid.grid_ymin;
            GRID_DELTAX           = my_grid.grid_deltax;
            GRID_DELTAY           = my_grid.grid_deltay;
            GRID_COLS             = my_grid.grid_cols;
            GRID_ROWS             = my_grid.grid_rows;
            %  1.3. OF THE LIKELIHOOD MODEL
            ZHIT       = this.zhit;
            SIGMA_HIT  = this.sigma_hit;
            SIGMA_HIT2 = SIGMA_HIT^2;
            %  1.4. MATRIX OF MINIMUM DISTANCES
            GRID_MAP_MIN_DISTANCE2     = my_grid.grid_map_min_distance2;
            GRID_MAP_MIN_DISTANCE_XBAR = my_grid.grid_min_distance_xbar;
            GRID_MAP_MIN_DISTANCE_YBAR = my_grid.grid_min_distance_ybar;
            
            %-----------------------------------------------------------%
            % 2. GET THE COMPONENTS OF THE POSE
            %-----------------------------------------------------------%
            xx = x(1);
            yy = x(2);
            theta = x(3);
            
            %-----------------------------------------------------------%
            % 3. MAIN CODE
            %-----------------------------------------------------------%
            %  3.1. SET SOME VALUES
            %   -> Initial value of the derivatives
            dx = 0;dy = 0;dt = 0;
            %   -> Set a(L14), and c(L16), KTE(L17-19)
            a = ZHIT*1/sqrt(2*pi*SIGMA_HIT2);
            c = (1-ZHIT)/MAX_RANGE;
            KTE = -1/(2*SIGMA_HIT2);
            %  3.2. MAIN LOOP
            LOG = 1;
            for i=1:KK_LIKELIHOOD
                % A. CHECK MAXIMUM RANGE MEASUREMENTS
                if(z(i) == MAX_RANGE)
                    if(DEBUG)
                        fprintf('   - %d measurement: Maximum range\n', i)
                        fprintf('      Do not take this mmeasurement\n')
                    end
                    continue
                end
                
                % B. COMPUTE THE END POINT OF THE MEASUREMENTS.
                xo = xx + xxk*cos(theta) - yyk*sin(theta) +...
                       z(i)*cos(theta + THETA_LASER_LIKELIHOOD(i) + tthk);         
                yo = yy + xxk*sin(theta) + yyk*cos(theta) + ...
                       z(i)*sin(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                %   -> Show messages for debug
                if(DEBUG)
                    fprintf('   - %d measurement: [xz:%2.4f, yz:%2.4f]\n', i, xz, yz)
                end
                
                % C. COMPUTE LINES 7, 8
                dev_xo_theta = -xxk*sin(theta) - yyk*cos(theta)...
                    - z(i)*sin(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                dev_yo_theta = -yyk*sin(theta) + xxk*cos(theta)...
                    + z(i)*cos(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                
                % D. COMPUTE THE INDEXES[x,y] THAT CORRESPOND TO THE
                %      LASER END POINTS IN THE GRID
                %   -> Get the indexes(1-based)
                idx = ceil((xo-GRID_XMIN )/GRID_DELTAX);
                idy = ceil((yo-GRID_YMIN )/GRID_DELTAY);
                %   -> Check for bottom left corner
                if(idx==0)
                    idx = idx + 1;
                end
                if(idy==0)
                    idy = idy + 1;
                end
                %   -> Show messages for debug
                if(DEBUG)
                    fprintf('     Grid index of end point[row, col] = [%d, %d] \n', idy, idx)
                end
                
                % E. COMPUTE THE CLOSEST OCCUPIED GRID
                if( (idx<1 || idx>GRID_COLS) || (idy<1 || idy>GRID_ROWS))
                    % IF THE END POINT FALLS BEYOND THE LIMITS OF THE GRID MAP
                    %   -> Show messages for debug
                    if(DEBUG)
                        fprintf('     This grid position for the end point is beyond limits\n')
                    end
                    continue
                else
                    % IF THE END FALLS WITHIN THE LIMITS OF THE GRID MAP
                    %   -> Get the minimum "PRE-COMPUTED" distance to an 
                    %      occupied space
                    dist2 = GRID_MAP_MIN_DISTANCE2(idy,idx);
                    xbar  = GRID_MAP_MIN_DISTANCE_XBAR(idy,idx);
                    ybar  = GRID_MAP_MIN_DISTANCE_YBAR(idy,idx);
                    % Check if the grid cell have not found an occupied cell
                    if(isnan(xbar))
                        %  Debug messages
                        if(DEBUG)
                            txt = ['OCC_GRID.GRADIENT_LIKELIHOOD_FIELD:'...
                                'Distance to an occupied grid is out of range'];
                            disp(txt)
                        end
                        continue
                    end
                    if(DEBUG)
                        fprintf('     Minimum distance2 = %2.4f \n', dist2)
                    end
                end
                
                % F. COMPUTE LINES 10-13
                dist2 = (xo-xbar)^2 + (yo-ybar)^2;
                dev_dist2_x = 2*(xo - xbar);
                dev_dist2_y = 2*(yo - ybar);
                dev_dist2_t = 2*( (xo-xbar)*dev_xo_theta  +...
                    (yo-ybar)*dev_yo_theta );
                
                % G. COMPUTE LINE 15
                b = -1/2*dist2/SIGMA_HIT2;
                
                % H. COMPUTE 17-19
                db_dx = KTE*dev_dist2_x;
                db_dy = KTE*dev_dist2_y;
                db_dt = KTE*dev_dist2_t;
                
                % I. COMPUTE 20-23
                log_q = log(a*exp(b) + c);
                cte2 = a*exp(b)/(c+a*exp(b));
                dlogq_dx = cte2*db_dx;
                dlogq_dy = cte2*db_dy;
                dlogq_dt = cte2*db_dt; 
                % Show messages for debug
                if(DEBUG)  
                    fprintf('     dlogq_dx = %e \n', dlogq_dx)
                    fprintf('     dlogq_dy = %e \n', dlogq_dy)
                    fprintf('     dlogq_dt = %e \n', dlogq_dt)
                end
                
                % J. ACCUMULATE THE DERIVATIVES
                dx = dx + dlogq_dx;
                dy = dy + dlogq_dy;
                dt = dt + dlogq_dt;
                LOG = LOG + log_q;
            end
        end
        
        function [dx, dy, dt, LOG] = ...
                gradient_likelihood_field_REAL(this, z, x, DEBUG)
            % occ_grid.gradient_likelihood_field_REAL Gradient of p(z|x)
            %
            %   GRADIENT_LIKELIHOOD_FIELD_REAL Computes the gradient of 
            %   the likelihood model of a laser range finder using REAL
            %   grid maps
            
            %-----------------------------------------------------------%
            % 1. GET PROPERTIES(To speed up)
            %-----------------------------------------------------------%
            %  1.1. SET MODE: 'MAPPER', 'NORMAL(localization)'
            mode = this.MODE_DISTANCES;
            if(strcmp(mode, 'KNOWN'))
                FLAG_MODE = 0;      % For localization
            elseif(strcmp(mode, 'UNKNOWN'))
                FLAG_MODE = 1;      % For mapping
            end
            %  1.2. OF THE LASER
            MAX_RANGE              = this.max_range;
            KK_LIKELIHOOD          = this.K_LIKELIHOOD;
            THETA_LASER_LIKELIHOOD = this.theta_laser_LIKELIHOOD;
            xxk                    = this.xk;
            yyk                    = this.yk;
            tthk                   = this.thk;
            %  1.3. OF THE GRID
            my_grid               = this.my_occ_grid;
            GRID_MAP              = my_grid.grid_map;
            GRID_XMIN             = my_grid.grid_xmin;
            GRID_YMIN             = my_grid.grid_ymin;
            GRID_DELTAX           = my_grid.grid_deltax;
            GRID_DELTAY           = my_grid.grid_deltay;
            GRID_COLS             = my_grid.grid_cols;
            GRID_ROWS             = my_grid.grid_rows;
            %  1.4. OF THE LIKELIHOOD MODEL
            ZHIT       = this.zhit;
            ZRAND      = this.zrand;
            SIGMA_HIT  = this.sigma_hit;
            SIGMA_HIT2 = SIGMA_HIT^2;
            %  1.5. GET MINIMUM DISTANCES (in localization)
            if(FLAG_MODE==0)
                GRID_MAP_MIN_DISTANCE2     = my_grid.grid_map_min_distance2;
                GRID_MAP_MIN_DISTANCE_XBAR = my_grid.grid_min_distance_xbar;
                GRID_MAP_MIN_DISTANCE_YBAR = my_grid.grid_min_distance_ybar;
            else
                my_grid.set_maximum_distance(0.4);
            end
            
            %-----------------------------------------------------------%
            % 2. GET THE COMPONENTS OF THE POSE
            %-----------------------------------------------------------%
            xx = x(1);
            yy = x(2);
            theta = x(3);
            
            %-----------------------------------------------------------%
            % 3. MAIN CODE
            %-----------------------------------------------------------%
            %  3.1. SET SOME VALUES
            %   -> Initial value of the derivatives
            dx = 0;dy = 0;dt = 0;
            %   -> Set a(L14), c(L16), and KTE(L17-19)
            a = ZHIT*(1/sqrt(2*pi*SIGMA_HIT2));
            c = ZRAND/MAX_RANGE;
            KTE = -1/(2*SIGMA_HIT2);
            %  3.2. MAIN LOOP
            LOG = 0;
            for i=1:KK_LIKELIHOOD
                % A. CHECK MAXIMUM RANGE MEASUREMENTS
                if(z(i) == MAX_RANGE)
                    if(DEBUG)
                        fprintf('   - %d measurement: Maximum range\n', i)
                        fprintf('      Do not take this mmeasurement\n')
                    end
                    continue
                end
                
                % B. COMPUTE THE END POINT OF THE MEASUREMENTS.
                xo = xx + xxk*cos(theta) - yyk*sin(theta) +...
                       z(i)*cos(theta + THETA_LASER_LIKELIHOOD(i) + tthk);         
                yo = yy + xxk*sin(theta) + yyk*cos(theta) + ...
                       z(i)*sin(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                %   -> Show messages for debug
                if(DEBUG)
                    fprintf('   - %d measurement: [xo:%2.4f, yo:%2.4f]\n', i,...
                        xo, yo)
                end
                
                % C. COMPUTE LINES 7, 8
                dev_xo_theta = -xxk*sin(theta) - yyk*cos(theta)...
                    - z(i)*sin(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                dev_yo_theta = -yyk*sin(theta) + xxk*cos(theta)...
                    + z(i)*cos(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                if(DEBUG)
                    fprintf('     [dev_xo_theta, dev_yo_theta]=[%2.4f, %2.4f]\n',...
                        dev_xo_theta, dev_yo_theta)
                end
                
                % D. COMPUTE THE INDEXES[x,y] THAT CORRESPOND TO THE
                %    LASER END POINTS IN THE GRID
                %   -> Get the indexes(1-based)
                idx = ceil((xo-GRID_XMIN)/GRID_DELTAX);
                idy = ceil((yo-GRID_YMIN)/GRID_DELTAY);
                %   -> Check for bottom left corner
                if(idx==0)
                    idx = idx + 1;
                end
                if(idy==0)
                    idy = idy + 1;
                end
                %   -> Show messages for debug
                if(DEBUG)
                    fprintf('     Grid index of laser end point[row, col] = [%d, %d] \n', idy, idx)
                end
                
                % E. COMPUTE THE CLOSEST OCCUPIED GRID - L9
                if( (idx<1 || idx>GRID_COLS) || (idy<1 || idy>GRID_ROWS) )
                    % IF THE END POINT FALLS BEYOND THE LIMITS OF THE GRID MAP
                    %   -> Show messages for debug
                    if(DEBUG)
                        fprintf('     This grid position for the end point is beyond limits\n')
                    end
                    continue
                else
                    % IF THE END FALLS WITHIN THE LIMITS OF THE GRID MAP
                    %   -> Get the minimum distance to an 
                    %      occupied space
                    if(GRID_MAP(idy, idx) == 1)
                        % 1st CASE: LASER END POINT IS OCCUPIED
                        if(DEBUG)
                            fprintf('     End point is OCCUPIED\n')
                        end
                        % Set default values
                        dist2 = 0;
                        xbar = xo;
                        ybar = yo;
                    else
                        % 2nd CASE: LASER END POINT IS FREE OR UNKNOWN
                        if(DEBUG)
                            fprintf('     End point is FREE or UNKNOWN\n')
                        end
                        
                        % Compute minimum distance to an occupied grid
                        if(FLAG_MODE==0)        % KNOWN mim distances
                            dist2 = GRID_MAP_MIN_DISTANCE2(idy,idx);
                            xbar  = GRID_MAP_MIN_DISTANCE_XBAR(idy,idx);
                            ybar  = GRID_MAP_MIN_DISTANCE_YBAR(idy,idx);
                            % Check if the grid cell have not found a occupied cell
                            if(isnan(xbar))
                                %  Debug messages
                                if(DEBUG)
                                    txt = ['rfinder.GRADIENT_LIKELIHOOD_FIELD:'...
                                        'Distance to an occupied grid is out of range'];
                                    disp(txt)
                                end
                                continue
                            end
                            
                        elseif(FLAG_MODE==1)    % UNKNOWN min distances
                            [dist2, xbar, ybar] = ...
                                my_grid.search_closest_grid_cell(xo, yo);               
                            % Check if the grid cell have not found a occupied cell
                            if(isnan(xbar))
                                %  Debug messages
                                if(DEBUG)
                                    txt = ['rfinder.GRADIENT_LIKELIHOOD_FIELD:'...
                                        'Maximun distance to occupied grid is out of range'];
                                    disp(txt)
                                end
                                continue
                            end
                        end
                    end
                    
                    % DEBUG MESSAGES
                    if(DEBUG)
                        fprintf('     Minimum distance2 = %2.4f \n', dist2)
                        fprintf('     [xbar, ybar] = [%2.4f, %2.4f]\n', xbar, ybar)
                        fprintf('     [xo, yo] = [%2.4f, %2.4f]\n', xo, yo)
                    end
                end
                
                % F. COMPUTE LINES 10-13
                dist2       = (xo-xbar)^2 + (yo-ybar)^2;
                dev_dist2_x = 2*(xo - xbar);
                dev_dist2_y = 2*(yo - ybar);
                dev_dist2_t = 2*( (xo-xbar)*dev_xo_theta  +...
                    (yo-ybar)*dev_yo_theta );
                if(DEBUG)
                    fprintf('     dist2 = %2.4f \n', dist2)
                    fprintf('     dev_dist2_ds = [%2.4f, %2.4f, %2.4f] \n',...
                        dev_dist2_x, dev_dist2_y, dev_dist2_t)
                end
                
                % G. COMPUTE LINE 15
                b = -1/2*dist2/SIGMA_HIT2;
                
                % H. COMPUTE 17-19
                db_dx = KTE*dev_dist2_x;
                db_dy = KTE*dev_dist2_y;
                db_dt = KTE*dev_dist2_t;
                
                % I. COMPUTE 20
                log_q = log(a*exp(b) + c);
                
                % J. COMPUTE 21-23
                cte2 = a*exp(b)/(c+a*exp(b));
                dlogq_dx = cte2*db_dx;
                dlogq_dy = cte2*db_dy;
                dlogq_dt = cte2*db_dt;         
                %   -> Show messages for debug
                if(DEBUG)  
                    fprintf('         cte2 = %e \n', cte2)
                    fprintf('     dlogq_dx = %e \n', dlogq_dx)
                    fprintf('     dlogq_dy = %e \n', dlogq_dy)
                    fprintf('     dlogq_dt = %e \n', dlogq_dt)
                end  
                
                % K. ACCUMULATE THE DERIVATIVES
                dx = dx + dlogq_dx;
                dy = dy + dlogq_dy;  
                dt = dt + dlogq_dt;  
                LOG = LOG + log_q;
            end
        end
        %--------------------------------------------------------------%
        
        function [XNEW, FLAGS, STEPS, Q] = likelihood_gradient_search_IDEAL(...
                this, s, x, DEBUG)
            % occ_grid.likelihood_gradient_search Gradient search
            %
            %   LIKELIHOOD_GRADIENT_SEARCH Finds the optimal pose that
            %   results of the maximization the log of 'p(z|x)' using the
            %   gradient search algorithm
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. SET PARAMETERS OF THE GRADIENT
            %   -> Learning rates
            alpha_pos = 3.0e-4;     % 0.005 (FULL=0.001)
            alpha_rot = 1.0e-3;     % 1e-2  (FULL=1e-3)
            %   -> Stoping criteria
            DELTA_DERIVATIVES    = 1e-4;   % For minimum derivatives
            DELTA_ERROR_POSITION = 5e-4;   % Delta for the position
            DELTA_ERROR_ANGLE    = 4e-3;
            %   -> Maximum number of steps
            MAX_STEPS = 150;
            %  1.2. DEFAULT VALUE OF THE DEBUG
            if(nargin<4)
                DEBUG = 0;
            end
            
            %-----------------------------------------------------------%
            % 2. LOOP OF SEARCH
            %-----------------------------------------------------------%
            %  2.1. ALLOCATE MEMORY
            Nposes = size(x,1);
            XNEW   = zeros(Nposes,3);
            FLAGS  = zeros(1,Nposes);
            STEPS  = zeros(1,Nposes);
            Q      = zeros(1,Nposes);
            %  2.1. LOOP
            for m = 1:Nposes
                % A. GET A POSE
                xsearch = x(m,:)';  % Convert to column
                
                % B. RESET FLAGS
                FLAG  = 0;
                steps = 0;
                
                % C. LOOP OF SEARCH FOR EACH TEST POSE
                for i=1:MAX_STEPS
                    % a. COMPUTE THE GRADIENT OF  'p(z|x)'
                    tSTART = tic;
                    [dx,dy,dt,logq] = this.gradient_likelihood_field(s, xsearch);
                    q = exp(logq);
                    t1 = toc(tSTART);
                    if(DEBUG)
                        fprintf('  Step: %d\n', i)
                        fprintf('   [dx, dy, dt] = [%e, %e, %e] \n', dx, dy, dt)
                        fprintf('   [logq, q] = [%e, %e] \n', logq, q)
                        fprintf('   Gradient time: %2.4f\n', t1)
                    end
                    
                    % b. GRADIENT SEARCH UPDATE
                    %   -> Update 'xsearch'
                    xsearch_new  = xsearch + alpha_pos*[dx;dy;0] + alpha_rot*[0;0;dt];
                    xsearch_new(3) = pi_to_pi(xsearch_new(3));
                    %   -> Update parameters
                    alpha_pos = alpha_pos/1.0;
                    alpha_rot = alpha_rot/1.02;
                    
                    % c. CHECK STOPPING CRITERIA
                    %   -> Check derivatives
                    FLAG_DERIVATIVES = abs(dx)<DELTA_DERIVATIVES &&...
                        abs(dy)<DELTA_DERIVATIVES && abs(dt)<DELTA_DERIVATIVES;
                    %   -> Get change in the pose      
                    delta_pose    = xsearch_new - xsearch;
                    delta_pose(3) = pi_to_pi(delta_pose(3));
                    %   -> Check flag of change in the pose
                    FLAG_DELTA_POSE = abs(delta_pose(1)) < DELTA_ERROR_POSITION &&...
                        abs(delta_pose(2)) < DELTA_ERROR_POSITION &&...
                        abs(delta_pose(3)) < DELTA_ERROR_ANGLE;
                    if(DEBUG)
                        fprintf('   delta pose: [%2.4f, %2.4f, %2.4f(%2.4f)]\n',...
                            delta_pose(1:2), delta_pose(3),delta_pose(3)*180/pi)
                        fprintf('   FLAG DERIVATIVES: %d\n', FLAG_DERIVATIVES)
                        fprintf('   FLAG POSE: %d\n', FLAG_DELTA_POSE)
                    end
                    %   -> Check stopping FLAGS
                    if(FLAG_DERIVATIVES || FLAG_DELTA_POSE)
                        % Messages
                        if(FLAG_DERIVATIVES || q>1e3)
                            % This is a succesful case
                            FLAG = 1;
                        end
                        break
                    end
                    
                    % d. UPDATE DATA
                    xsearch = xsearch_new;
                    steps = steps + 1;
                end
                
                % SAVE RESULTS
                XNEW(m,:) = xsearch;
                FLAGS(m)  = FLAG;
                STEPS(m)  = steps;
                Q(m)      = q;
            end
        end
        
        function [XNEW, FLAGS, STEPS, Q] = likelihood_gradient_search_REAL(this,...
                s, X)
            % rfinder.likelihood_gradient_search Gradient search
            %
            %   LIKELIHOOD_GRADIENT_SEARCH Finds the optimal pose that
            %   results of the maximization the log of 'p(z|x)' using the
            %   gradient search algorithm
            
            %-----------------------------------------------------------%
            % 1. GET PROPERTIES(To speed up)
            %-----------------------------------------------------------%
            %   -> Of the grid('MAPPER', 'NORMAL(localization)')
            mode = this.MODE_DISTANCES;
            if(strcmp(mode, 'KNOWN'))
                FLAG_MODE = 0;      % For localization
            elseif(strcmp(mode, 'UNKNOWN'))
                FLAG_MODE = 1;      % For mapping
            end
            %   -> Of the laser
            MAX_RANGE              = this.max_range;
            KK_LIKELIHOOD          = this.K_LIKELIHOOD;
            THETA_LASER_LIKELIHOOD = this.theta_laser_LIKELIHOOD;
            xxk                    = this.xk;
            yyk                    = this.yk;
            tthk                   = this.thk;
            %   -> Of the grid
            my_grid               = this.my_occ_grid;
            GRID_MAP              = my_grid.grid_map;
            GRID_XMIN             = my_grid.grid_xmin;
            GRID_YMIN             = my_grid.grid_ymin;
            GRID_DELTAX           = my_grid.grid_deltax;
            GRID_DELTAY           = my_grid.grid_deltay;
            GRID_COLS             = my_grid.grid_cols;
            GRID_ROWS             = my_grid.grid_rows;
            %   -> Of the likelihood model
            ZHIT       = this.zhit;
            ZRAND      = this.zrand;
            SIGMA_HIT  = this.sigma_hit;
            SIGMA_HIT2 = SIGMA_HIT^2;
            %   -> Get the matrix on minimum distances in localization
            if(FLAG_MODE==0)
                %GRID_MAP_MIN_DISTANCE2     = my_grid.grid_map_min_distance2;
                GRID_MAP_MIN_DISTANCE_XBAR = my_grid.grid_min_distance_xbar;
                GRID_MAP_MIN_DISTANCE_YBAR = my_grid.grid_min_distance_ybar;
            end
            %  1.4. SET MINIMUM DISTANCE THRESHOLD
            if(strcmp(this.MODE_DISTANCES, 'UNKNOWN'))
                my_grid.set_maximum_distance(0.4);
            end
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1. SET PARAMETERS OF THE GRADIENT
            %   -> Learning rates
            if(FLAG_MODE==1)  % UNKNOWN min distances - According to ROS
                alpha_pos = 1.5e-4;     %
                alpha_rot = 1.0e-4;     %
            else
                alpha_pos = 3.0e-4;     %
                alpha_rot = 1.0e-3;     %
            end
            %   -> Stoping criteria
            DELTA_DERIVATIVES    = 1e-4;   % For minimum derivatives
            DELTA_ERROR_POSITION = 5e-4;   % Delta for the position
            DELTA_ERROR_ANGLE    = 3e-3;
            %   -> Maximum number of steps
            MAX_STEPS = 150;
            %  3.2. FOR THE CALCULUS OF THE GRADIENT
            %   -> Set a(L14), c(L16), and KTE(L17-19)
            a = ZHIT*(1/sqrt(2*pi*SIGMA_HIT2));
            c = ZRAND/MAX_RANGE;
            KTE = -1/(2*SIGMA_HIT2);
            
            %-----------------------------------------------------------%
            % 3. LOOP OF SEARCH
            %-----------------------------------------------------------%
            %  3.1. ALLOCATE MEMORY
            Nposes = size(X,1);
            XNEW   = zeros(Nposes,3);
            FLAGS  = zeros(Nposes,1);
            STEPS  = zeros(Nposes,1);
            Q      = zeros(Nposes,1);
            %  3.2. LOOP
            for m = 1:Nposes
                % GET A POSE
                xsearch = X(m,:)';  % Convert it to a column
                % RESET FLAGS
                FLAG  = 0;
                steps = 0;
                
                % LOOP OF GRADIENT SEARCH FOR EACH TEST POSE
                for sss=1:MAX_STEPS
                    %---------------------------------------------------%
                    % A. GET THE COMPONENTS OF THE POSE
                    %---------------------------------------------------%
                    xx    = xsearch(1);
                    yy    = xsearch(2);
                    theta = xsearch(3);
                    
                    %---------------------------------------------------%
                    % B. COMPUTE THE GRADIENT OF  'p(z|x)'
                    %---------------------------------------------------%
                    %  a. SET SOME VALUES
                    %   -> Initial value of the derivatives
                    dx = 0;dy = 0;dt = 0;
                    %  b. MAIN LOOP
                    LOG = 0;
                    for i=1:KK_LIKELIHOOD
                        % CHECK MAXIMUM RANGE MEASUREMENTS
                        if(s(i) == MAX_RANGE)
                            continue
                        end
                        
                        % COMPUTE THE END POINT OF THE MEASUREMENTS.
                        xo = xx + xxk*cos(theta) - yyk*sin(theta) +...
                            s(i)*cos(theta + THETA_LASER_LIKELIHOOD(i) + tthk);         
                        yo = yy + xxk*sin(theta) + yyk*cos(theta) + ...
                            s(i)*sin(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                        
                        % COMPUTE LINES 7, 8
                        dev_xo_theta = -xxk*sin(theta) - yyk*cos(theta)...
                            - s(i)*sin(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                        dev_yo_theta = -yyk*sin(theta) + xxk*cos(theta)...
                            + s(i)*cos(theta + THETA_LASER_LIKELIHOOD(i) + tthk);
                        
                        % COMPUTE THE INDEXES[x,y] THAT CORRESPOND TO THE
                        % LASER END POINTS IN THE GRID
                        %   -> Get the indexes(1-based)
                        idx = ceil((xo-GRID_XMIN)/GRID_DELTAX);
                        idy = ceil((yo-GRID_YMIN)/GRID_DELTAY);
                        %idx
                        % idy
                        %   -> Check for bottom left corner
                        if(idx==0)
                            idx = idx + 1;
                        end
                        if(idy==0)
                            idy = idy + 1;
                        end
                        
                        % COMPUTE THE CLOSEST OCCUPIED GRID - L9
                        if( (idx<1 || idx>GRID_COLS) || (idy<1 || idy>GRID_ROWS))
                            % IF THE END POINT FALLS BEYOND THE LIMITS OF THE GRID MAP
                            continue
                        else
                            % IF THE END FALLS WITHIN THE LIMITS OF THE GRID MAP
                            %   -> Get the minimum distance to an 
                            %      occupied space
                            if(GRID_MAP(idy, idx) == 1)
                                % 1st CASE: LASER END POINT IS OCCUPIED
                                % Set default values
                                %dist2 = 0;
                                xbar = xo;
                                ybar = yo;
                               	
                            elseif(GRID_MAP(idy, idx) == 0.0)
                                % 3rd CASE: LASER END POINT IS FREE
                                % Compute minimum distance to an occupied grid
                                if(FLAG_MODE==0)        % KNOWN
                                    %dist2 = GRID_MAP_MIN_DISTANCE2(idy,idx);
                                    xbar  = GRID_MAP_MIN_DISTANCE_XBAR(idy,idx);
                                    ybar  = GRID_MAP_MIN_DISTANCE_YBAR(idy,idx);
                                    % Check if the grid cell have not found a occupied cell
                                    if(isnan(xbar))
                                        continue
                                    end
                                    
                                elseif(FLAG_MODE==1)    % UNKNOWN
                                    [~, xbar, ybar] = ...
                                        my_grid.search_closest_grid_cell(xo, yo);
                                    % Check if the grid cell have not found a occupied cell
                                    if(isnan(xbar))
                                        continue
                                    end
                                end
                                
                            else %if(GRID_MAP(idy, idx) == 0.5)
                                % 2nd CASE: LASER END POINT IS UNKNOWN OR FREE
                                % Compute minimum distance to an occupied grid
                                if(FLAG_MODE==0)        % Localization
                                    %dist2 = GRID_MAP_MIN_DISTANCE2(idy,idx);
                                    xbar  = GRID_MAP_MIN_DISTANCE_XBAR(idy,idx);
                                    ybar  = GRID_MAP_MIN_DISTANCE_YBAR(idy,idx);
                                    % Check if the grid cell have not found a occupied cell
                                    if(isnan(xbar))
                                        continue
                                    end
                                    
                                elseif(FLAG_MODE==1)    % Mapper
                                    [~, xbar, ybar] = ...
                                        my_grid.search_closest_grid_cell(xo,yo);
                                    % Check if the grid cell have not found a occupied cell
                                    if(isnan(xbar))
                                        continue
                                    end
                                end
                            end
                        end
                        
                        % COMPUTE LINES 10-13
                        dist2       = (xo-xbar)^2 + (yo-ybar)^2;
                        dev_dist2_x = 2*(xo - xbar);
                        dev_dist2_y = 2*(yo - ybar);
                        dev_dist2_t = 2*( (xo-xbar)*dev_xo_theta  +...
                            (yo-ybar)*dev_yo_theta );
                        
                        % COMPUTE LINE 15
                        b = -1/2*dist2/SIGMA_HIT2;
                        
                        % COMPUTE 17-19
                        db_dx = KTE*dev_dist2_x;
                        db_dy = KTE*dev_dist2_y;
                        db_dt = KTE*dev_dist2_t;
                        
                        % COMPUTE 20
                        log_q = log(a*exp(b) + c);
                        
                        % COMPUTE 21-23
                        cte2 = a*exp(b)/(c+a*exp(b));
                        dlogq_dx = cte2*db_dx;
                        dlogq_dy = cte2*db_dy;
                        dlogq_dt = cte2*db_dt; 
                        
                        % ACCUMULATE THE DERIVATIVES
                        dx = dx + dlogq_dx;
                        dy = dy + dlogq_dy;  
                        dt = dt + dlogq_dt;  
                        LOG = LOG + log_q;
                    end
                    
                    % GET RESULTS
                    logq = LOG;
                    q = exp(logq);
                    
                    %---------------------------------------------------%
                    % C. GRADIENT SEARCH UPDATE
                    %---------------------------------------------------%
                    %   -> Update 'xsearch'
                    xsearch_new    = xsearch + alpha_pos*[dx;dy;0] + alpha_rot*[0;0;dt];
                    xsearch_new(3) = pi_to_pi(xsearch_new(3));
                    %   -> Update parameters
                    alpha_pos = alpha_pos/1.005;
                    alpha_rot = alpha_rot/1.015;
                    
                    %---------------------------------------------------%
                    % D. CHECK STOPPING CRITERIA
                    %---------------------------------------------------%
                    %   -> Check derivatives
                    FLAG_DERIVATIVES = abs(dx)<DELTA_DERIVATIVES &&...
                        abs(dy)<DELTA_DERIVATIVES && abs(dt)<DELTA_DERIVATIVES;
                    %   -> Get change in the pose      
                    delta_pose    = xsearch_new - xsearch;
                    delta_pose(3) = pi_to_pi(delta_pose(3));
                    %   -> Check flag of change in the pose
                    FLAG_DELTA_POSE = abs(delta_pose(1)) < DELTA_ERROR_POSITION &&...
                        abs(delta_pose(2)) < DELTA_ERROR_POSITION &&...
                        abs(delta_pose(3)) < DELTA_ERROR_ANGLE;
                    %   -> Check stopping FLAGS
                    if(FLAG_DERIVATIVES || FLAG_DELTA_POSE)
                        % Messages
                        if(FLAG_DERIVATIVES || q>1e3)
                            % This is a succesful case
                            FLAG = 1;
                        end
                        break
                    end
                    
                    %---------------------------------------------------%
                    % E. UPDATE DATA
                    %---------------------------------------------------%
                    xsearch = xsearch_new;
                    steps = steps + 1;
                end
                
                % SAVE RESULTS
                XNEW(m,:) = xsearch;
                FLAGS(m)  = FLAG;
                STEPS(m)  = steps;
                Q(m)      = q;
            end
        end
    end
    
    %------------------------------------------------------------------%
    %  STATIC METHODS
    %------------------------------------------------------------------%
    methods(Static)
        function credits()
            % rfinder.credits Show credits
            %
            %   CREDITS Show credits
            
            %-----------------------------------------------------------%
            % 1. MESSAGES
            %-----------------------------------------------------------%
            disp('--------------------------------------')
            disp('---  ABOUT THE "rfinder"  LIBRARY  ---')
            disp('--------------------------------------')
            disp('  Author: Ivan A. Calle Flores')
            disp('  e-mail: ivan.calle.flores@gmail.com')
        end
    end
end

