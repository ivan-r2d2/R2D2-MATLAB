% ROBOT Differential-like drive robot class
%
%   This class implements the motion models described in the book
%   "Probabilistic Robotics. S. Thrun" and also those described in: 
%
classdef robot < handle
    %------------------------------------------------------------------%
    %  DECLARE PROPERTIES WITH THE FOLLOWING ATTRIBUTES
    %   - Can only be accesed inside the class
    %   - Can be read from anywhere
    %------------------------------------------------------------------%
    properties(SetAccess=private, GetAccess=public)     
        % MAIN PROPERTIES
        x                   % Pose [x;y;th]
        x_ODOMETRY          % Odometry pose [x;y;th]
        u_ODOMETRY          % Vector of odometry controls [drot1;dtrans;drot2]
        MOTION_MODEL        % 'STANDARD', 'STANDARD2', 'IMPROVED', 'GAUSSIAN'
        LEVEL_NOISE         % Level of noise (-1 when set manually by the user)
        NAME_ROBOT          % 'R2D2-00', 'R2D2-R1,', 'R2D2-R2,' etc.
                            % This is just used to plot the real robot
        DTRANS_ROT = 0.010; % Maximum value of "dtrans" to be considered as a
                            % pure rotation in the 'IMPROVED' and 'GAUSSIAN'
                            % odometry models
        
        % MAIN MECHANICAL DIMENSIONS
        D                   % Diameter of the wheels(m)
        r                   % Radious of the wheels(m)
        B                   % Distance from each wheel to the center point(m)
        
        % MECHANICAL PROPERTIES OF OUR REAL ROBOT
        DIMENSIONS          % Structure
        
        % MOTION ERROR PARAMETERS
        alpha_VELOCITY      % For the velocity model
        alpha_ODOMETRY      % For the odometry model
        
        % GAUSSIAN APPROXIMATION FOR THE PREDICTED BELIEF
        % This is the prediction step of the EKF  
        MU                  % Mean [x;y;th]
        PP                  % Covariance matrix
        
        % STORE DATA
        data                % Structure of MCL data results
        CHUNK_DATA          % Size of CHUNK
        FLAG_STORAGE = 0    % 0:Not ready/1:Ready to save data
    end
    
    
    %------------------------------------------------------------------%
    %   PUBLIC METHODS
    %------------------------------------------------------------------%
    methods
        %---------------------- CONSTRUCTOR -------------------------%
        function robot = robot(FLAG)
            % robot.robot Constructor function
            %
            %   ROBOT(FLAG) returns a valid "robot" object. The argument
            %   'FLAG' determines if it's going to appear the 'welcome'
            %   message box.
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. SET DEFAULT VALUE OF 'FLAG'
            if(nargin==0)
                FLAG = 0;   % By default we don't show message box
            end
            
            %-----------------------------------------------------------%
            % 2. DISPLAY MESSAGE BOX
            %-----------------------------------------------------------%
            SHOW_MSGBOX = 0;            % FLAG TO DISPLAY MSG BOX
            if(SHOW_MSGBOX || FLAG)
                % 2.1. GET PATH OF 'robot' m-FILE
                robotpath = fileparts( mfilename('fullpath') );
                % 2.2. LOAD ICON
                str = [robotpath '/R2D2-00.png'];
                robot_icon = imread(str);
                % 2.3. DISPLAY MESSAGE WINDOW
                color = (0:255)/255;
                color = [color(:) color(:) color(:)];
                h = msgbox({'You are using the R2D2 library of robot motion...!!!',...
                    'Author: Ivan A. Calle Flores', 'e-mail: ivan.calle.flores@gmail.com'},...
                    'R2D2 Motion library',...
                    'custom', robot_icon, color,...
                    'modal');
                uiwait(h)
            end
        end
        
        function set_robot(this, name)
            % robot.set_robot Set the real robot
            %
            %   SET_ROBOT Set the real robot to be used to do the
            %             simulations
            %
            % INPUT
            %   name: Type of robot ('R2D2-00', 'R2D2-R1', etc)
            
            %-----------------------------------------------------------%
            % SET THE TYPE OF ROBOT
            %-----------------------------------------------------------%
            switch(name)
                case {'R2D2-00', 'R2D2-01'}
                    this.NAME_ROBOT = name;
                    this.use_robot_R2D2();
                case {'R2D2-R1', 'R2D2-R2'}
                    this.NAME_ROBOT = name;
                    this.use_robot_R2D2_RX();
                otherwise
                    error('robot.set_robot: Undefined name of robot')
            end
        end
        
        function set_motion_model(this, model)
            % robot.set_motion_model Set the motion model
            %
            %   SET_MOTION_MODEL Sets the motion model to be used
            %
            % INPUT
            %   model: 'STANDARD', 'STANDARD2', 'IMPROVED', 'GAUSSIAN'
            
            switch(model)
                case {'STANDARD', 'STANDARD2', 'IMPROVED', 'GAUSSIAN'}
                    this.MOTION_MODEL = model;
                otherwise
                    error('robot.set_motion_model: Undefined motion model')
            end
        end
        
        function set_level_of_noise(this, LEVEL_NOISE)
            % robot.set_level_of_noise
            %
            %   SET_LEVEL_OF_NOISE Set the level of motion noise
                        
            %-----------------------------------------------------------%
            % 1. SET VELOCITY NOISE PARAMETERS
            %-----------------------------------------------------------%
            %  1.1. GET SELECTED PARAMETERS
            switch LEVEL_NOISE
                case 1
                    % trans: STD_X = 1cm, STD_Y = 2cm, STD_th = 2.31°
                    %   rot: STD_th = 2.54°
                    alpha1 = 5.00e-4;        alpha2 = 1.00e-4;
                    alpha3 = 8.00e-3;        alpha4 = 3.50e-3;
                    alpha5 = 1.00e-4;        alpha6 = 5.00e-4;
                case 2
                    % trans: STD_X = 2cm, STD_Y = 4cm, STD_th= 4.59°
                    %   rot: STD_th = 5.07°
                    alpha1 = 2.00e-3;        alpha2 = 2.00e-4;
                    alpha3 = 3.20e-2;        alpha4 = 1.55e-2;
                    alpha5 = 1.00e-4;        alpha6 = 5.00e-4;
                case 3
                    % trans: STD_X = 3cm, STD_Y = 6cm, STD_th = 7.00°
                    %   rot: STD_th = 7.55°
                    alpha1 = 4.50e-3;        alpha2 = 3.00e-4;
                    alpha3 = 7.50e-2;        alpha4 = 3.50e-2;
                    alpha5 = 1.00e-4;        alpha6 = 5.00e-4;
                case 4
                    % trans: STD_X = 4cm, STD_Y = 8cm, STD_th = 9.24°
                    %   rot: STD_th = 10.06°
                    alpha1 = 8.00e-3;        alpha2 = 4.00e-4;
                    alpha3 = 1.30e-1;        alpha4 = 6.20e-2;
                    alpha5 = 1.00e-4;        alpha6 = 5.00e-4;
                case 5
                    % STD_X = 5cm, STD_Y = 10cm, STD_th = 12.00°
                    % STD_th = 12.51°
                    alpha1 = 1.21e-2;        alpha2 = 5.00e-4;      
                    alpha3 = 2.20e-1;        alpha4 = 9.65e-2;      
                    alpha5 = 1.00e-4;        alpha6 = 5.00e-4;
                case 6
                    % STD_X = 6cm, STD_Y = 12cm, STD_th = 13.97°
                    % STD_th = 15.08°
                    alpha1 = 1.75e-2;        alpha2 = 6.00e-4;
                    alpha3 = 2.95e-1;        alpha4 = 1.40e-1;      
                    alpha5 = 1.00e-4;        alpha6 = 5.00e-4;
                case 7
                    % STD_X = 7cm, STD_Y = 14cm, STD_th = 16.31°
                    % STD_th = 17.55°
                    alpha1 = 2.35e-2;        alpha2 = 7.00e-4;      
                    alpha3 = 4.10e-1;        alpha4 = 1.90e-1;      
                    alpha5 = 1.00e-4;        alpha6 = 5.00e-4;
                case 8
                    % STD_X = 8cm, STD_Y = 16cm, STD_th = 18.77°
                    % STD_th = 20.15°
                    alpha1 = 3.05e-2;        alpha2 = 8.00e-4;      
                    alpha3 = 5.42e-1;        alpha4 = 2.50e-1;      
                    alpha5 = 1.00e-4;        alpha6 = 5.00e-4;
                case 9
                    % STD_X = 9cm, STD_Y = 18cm, STD_th = 21.28°
                    % STD_th = 22.56
                    alpha1 = 3.75e-2;        alpha2 = 9.00e-4;      
                    alpha3 = 6.90e-1;        alpha4 = 3.14e-1;      
                    alpha5 = 1.00e-4;        alpha6 = 5.00e-4;
                case 10
                    % STD_X = 10cm, STD_Y = 20cm, STD_th = 24.09°
                    % STD_th = 25.24°
                    alpha1 = 4.45e-2;        alpha2 = 10.0e-4;      
                    alpha3 = 8.80e-1;        alpha4 = 3.90e-1;      
                    alpha5 = 1.00e-4;        alpha6 = 5.00e-4;
                otherwise
                    str = ['robot.set_level_of_noise:',...
                        'The level of noise is not defined'];
                    error(str)
            end
            % 1.2. SET NOISE PARAMETERS 
            ALPHA = [alpha1;alpha2;alpha3;alpha4;alpha5;alpha6];
            this.alpha_VELOCITY = ALPHA;
            
            
            %-----------------------------------------------------------%
            % 2. SET ODOMETRY NOISE PARAMETERS
            %-----------------------------------------------------------%
            %  2.1. GET SELECTED PARAMETERS
            switch LEVEL_NOISE
                case 1
                    % TRANS: STD_X = 0.5cm, STD_Y = 1.0cm, STD_TH = 0.81°
                    %   ROT: STD_THETA = 1.5°
                    alpha1 = 4.40e-4;         alpha2 = 1.00e-4;
                    alpha3 = 2.50e-5;         alpha4 = 2.00e-5;
                case 2
                    % TRANS: STD_X = 1.0cm, STD_Y = 2.0cm, STD_TH = 1.62°
                    %   ROT: STD_THETA = 3.0°
                    alpha1 = 1.80e-3;         alpha2 = 4.00e-4;      
                    alpha3 = 1.00e-4;         alpha4 = 4.00e-5;
                case 3
                    % TRANS: STD_X = 1.5cm, STD_Y = 3.0cm, STD_TH = 2.43°
                    %   ROT: STD_THETA = 4.5°
                    alpha1 = 4.00e-3;         alpha2 = 9.05e-4;      
                    alpha3 = 2.25e-4;         alpha4 = 6.00e-5;
                case 4
                    % TRANS: STD_X = 2.0cm, STD_Y = 4.0cm, STD_TH = 3.22°
                    %   ROT: STD_THETA = 6.0°
                    alpha1 = 7.00e-3;         alpha2 = 1.60e-3;      
                    alpha3 = 4.00e-4;         alpha4 = 8.00e-5;
                case 5
                    % TRANS: STD_X = 2.5cm, STD_Y = 5.0cm, STD_TH = 4.05°
                    %   ROT: STD_THETA = 7.5°
                    alpha1 = 1.10e-2;         alpha2 = 2.50e-3;
                    alpha3 = 6.25e-4;         alpha4 = 1.00e-4;
                case 6
                    % TRANS: STD_X = 3.0cm, STD_Y = 6.0cm, STD_TH = 4.88°
                    %   ROT: STD_THETA = 9.0°
                    alpha1 = 1.60e-2;         alpha2 = 3.65e-3; 
                    alpha3 = 8.95e-4;         alpha4 = 1.30e-4;
                case 7
                    % TRANS: STD_X = 3.5cm, STD_Y = 7.0cm, STD_TH = 5.75°
                    %   ROT: STD_THETA = 10.5°
                    alpha1 = 2.15e-2;         alpha2 = 5.00e-3;
                    alpha3 = 1.25e-3;         alpha4 = 1.60e-4;
                case 8
                    % TRANS: STD_X = 4.0cm, STD_Y = 8.0cm, STD_TH = 6.52°
                    %   ROT: STD_THETA = 12.0°
                    alpha1 = 2.80e-2;         alpha2 = 6.50e-3;
                    alpha3 = 1.60e-3;         alpha4 = 1.90e-4;
                case 9
                    % TRANS: STD_X = 4.5cm, STD_Y = 9.0cm, STD_TH = 7.38°
                    %   ROT: STD_THETA = 13.5°
                    alpha1 = 3.56e-2;         alpha2 = 8.20e-3;
                    alpha3 = 2.00e-3;         alpha4 = 2.20e-4;
                case 10
                    % TRANS: STD_X = 5.0cm, STD_Y = 10.0cm, STD_TH = 8.13°
                    %   ROT: STD_THETA = 15.0°
                    alpha1 = 4.45e-2;         alpha2 = 1.02e-2;
                    alpha3 = 2.50e-3;         alpha4 = 2.50e-4;
                otherwise
                    str = ['robot.set_level_of_noise:',...
                        'The level of noise is not defined'];
                    error(str)
            end
            % 2.2. SET NOISE PARAMETERS 
            ALPHA = [alpha1;alpha2;alpha3;alpha4];
            this.alpha_ODOMETRY = ALPHA;
            
            
            %-----------------------------------------------------------%
            % 3. SAVE LEVEL OF NOISE
            %-----------------------------------------------------------%
            this.LEVEL_NOISE = LEVEL_NOISE;
        end
        %-------------------- END CONSTRUCTOR -----------------------%
        
        
        %--------------- BASIC ROBOT FUNCTIONS ----------------------%
        function set_pose(this, x)
            % robot.set_pose Set the pose of the robot
            %
            %   SET_POSE(X) Set the new pose of the robot
            %
            % INPUT
            %   x: Pose of the robot
            %
            % NOTES: 
            %   - This method ensures that the orientation is in the
            %     correct range (-pi,pi]
            %   - The pose is saved as a column
            
            %-----------------------------------------------------------%
            % 1. ARGUMENT'S CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK DIMENSION
            if(length(x) ~= 3)
                error('robot.set_pose: Pose must be of length 3')
            end
            %  1.2. MAKE SURE THE POSE IS IN COLUMN FORMAT
            x = x(:);
            
            %-----------------------------------------------------------%
            % 2. SET THE POSE
            %-----------------------------------------------------------%
            %  2.1. CHECK THE ORIENTATION
            x(3) = pi_to_pi(x(3));
            %  2.2. SET POSE
            this.x = x;
        end
        
        function set_odometry_pose(this, x_odometry)
            % robot.set_odometry_pose Set the odometry pose
            %
            %   SET_ODOMETRY_POSE Set the "internal" odometry pose
            
            %-----------------------------------------------------------%
            % 1. ARGUMENT'S CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK DIMENSION
            if(length(x_odometry) ~= 3)
                error('robot.set_odometry_pose: Pose must be of length 3')
            end
            %  1.2. MAKE SURE THE POSE IS IN COLUMN FORMAT
            x_odometry = x_odometry(:);
            
            %-----------------------------------------------------------%
            % 2. SET THE POSE
            %-----------------------------------------------------------%
            %  2.1. MAKE SURE THE ORIENTATION IS CORRECT
            x_odometry(3) = pi_to_pi(x_odometry(3));
            %  2.2. SET ODOMETRY POSE
            this.x_ODOMETRY = x_odometry;
        end
             
        function display_pose(this, FLAG)
            % robot.display_pose  Display the pose of the robot
            %
            %   DISPLAY_POSE Display the "current" pose of the robot
            %   in the Command Window
            % 
            % INPUTS
            %   FLAG: Flag to set orientation
            %           0: Radian (default)
            %           1: Sexagesimal
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. CHECK THE POSE
            if(isempty(this.x))
                error('robot.display_pose: The pose is not defined')
            end
            %  1.2. DEFAULT FLAG VALUE
            if(nargin < 2)
                FLAG = 0;
            end
            
            %-----------------------------------------------------------%
            % 2. DISPLAY THE POSE
            %-----------------------------------------------------------%
            fprintf(' The pose of ''%s'' is:\n', inputname(1))
            pose = this.x;
            if(FLAG == 0)
                fprintf('   [x(m), y(m), theta(rad)] = [%2.4f, %2.4f, %2.4f]\n',...
                    pose)
            else
                fprintf('   [x(m), y(m), theta(sexag)] = [%2.4f, %2.4f, %2.4f]\n',...
                    pose(1:2), pose(3)*180/pi)
            end
        end
        
        function display_odometry_pose(this, FLAG)
            % robot.display_odometry_pose  Display the odometry pose
            %
            %   DISPLAY_ODOMETRY_POSE Display the "current" odometry
            %   pose in the Command Window
            % 
            % INPUTS
            %   FLAG: Flag to set orientation
            %           0: Radian(default)
            %           1: Sexagesimal
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. CHECK THE POSE
            if(isempty(this.x_ODOMETRY))
                str = ['robot.display_odometry_pose:',...
                    'The odometry pose is not defined'];
                error(str)
            end
            %  1.2. DEFAULT FLAG VALUE
            if(nargin < 2)
                FLAG = 0;
            end
            
            %-----------------------------------------------------------%
            % 2. DISPLAY THE POSE
            %-----------------------------------------------------------%
            fprintf(' The odometry pose of ''%s'' is:\n', inputname(1))
            pose = this.x_ODOMETRY;
            if(FLAG == 0)
                fprintf('   [x(m), y(m), theta(rad)] = [%2.4f, %2.4f, %2.4f]\n',...
                    pose)
            else
                fprintf('   [x(m), y(m), theta(sexag)] = [%2.4f, %2.4f, %2.4f]\n',...
                    pose(1:2), pose(3)*180/pi)
            end
        end
               
        function set_mechanical_parameters(this, D, B)
            % robot.set_mechanical_parameters Set mechanical parameters 
            %                           
            %   SET_MECHANICAL_PARAMETERS(D, B) Set the "main" mechanical
            %   parameters of the robot
            %
            % OUTPUTS
            %   The calling object with the new mechanical parameters
            %
            % INPUTS
            %   D: Diameter of the wheels
            %   B: Distance from each wheel to the center point
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 3)
                str = ['robot.set_mechanical_parameters:', ...
                    'It is neccesary 2 inputs'];
                error(str)
            end
            %  1.2. CHECK VALUES OF THE PARAMETERS
            if(D<=0 || B<=0)
                str = ['robot.set_mechanical_parameters:', ...
                    'Invalid values of the arguments'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. SET PROPERTIES
            %-----------------------------------------------------------%
            %  2.1. SET DIAMETER AND THE RADIUS OF THE WHEEL
            this.D = D;
            this.r = D/2;
            %  2.2. SET DISTANCE FROM EACH WHEEL TO THE CENTER POINT
            this.B = B;
        end
        
        function [v, w] = compute_robot_velocities(this,wm1,wm2)
            % robot.compute_robot_velocities Compute the robot velocities
            %
            %   COMPUTE_ROBOT_VELOCITIES(WM1,WM2) Computes the robot
            %   velocities from the motor velocities.
            %
            % OUTPUTS
            %   v: Translational velocity
            %   w: Rotational velocity
            %
            % INPUTS
            %   w_motor1: Angular velocity of the right motor "motor 1"
            %   w_motor2: Angular velocity of the left motor "motor 2"
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            if(nargin < 3)
                str = ['robot.compute_robot_velocities:',...
                    'Insufficient number of arguments'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. CHECK AND GET PROPERTIES
            %-----------------------------------------------------------%
            %  2.1. CHECK EXISTENCE OF "B"
            if(isempty(this.B))
                error('robot.compute_robot_velocities: B is not defined')
            end
            %  2.2. CHECK EXISTENCE OF "r"
            if(isempty(this.r))
                error('robot.compute_robot_velocities: r is not defined')
            end
            %  2.3. GET MECHANICAL PROPERTIES
            rr = this.r;
            BB = this.B;
            
            %-----------------------------------------------------------%
            % 3. COMPUTE ROBOT VELOCITIES
            %-----------------------------------------------------------%
            %  3.1 TRANSLATIONAL VELOCITY
            v = 0.5*rr*(wm1 + wm2);
            %  3.2. ROTATIONAL VELOCITY
            w = 0.5*(rr/BB)*(wm1 - wm2);
        end 
        
        function [w_motor1, w_motor2] = compute_motor_velocities(this,v,w)
            % robot.compute_motor_velocities Compute the angular velocities
            %                                of the motors
            %
            %  COMPUTE_MOTOR_VELOCITIES(V,W) Computes the corresponding 
            %  angular motor velocities for the given robot velocities.
            %
            % OUTPUTS
            %   w_motor1: Angular velocity of the right motor "motor 1"
            %   w_motor2: Angular velocity of the left motor "motor 2"
            %
            % INPUTS
            %   v: Translational velocity(m/s)
            %   w: Rotational velocity(rad/s)
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 3)
                str = ['robot.compute_motor_velocities:',...
                    'Insufficient number of arguments'];
                error(str)
            end
            %  1.2. CHECK EXISTENCE OF "B"
            if(isempty(this.B))
                error('robot.compute_motor_velocities: B is not defined')
            end
            %  1.3. CHECK EXISTENCE OF "r"
            if(isempty(this.r))
                error('robot.compute_motor_velocities: r is not defined')
            end
            %  1.4. GET PROPERTIES
            rr = this.r;
            BB = this.B;
            
            %-----------------------------------------------------------%
            % 2. COMPUTE MOTOR VELOCITIES
            %-----------------------------------------------------------%
            %  2.1. VELOCITY OF THE RIGHT MOTOR "motor 1"
            w_motor1 = (v + BB*w)/rr;
            %  2.2. VELOCITY OF THE LEFT MOTOR "motor 2"
            w_motor2 = (v - BB*w)/rr;
        end             
        %--------------- END MOTION FUNCTIONS -----------------------%
        
        
        %-------------------- PLOTTING FUNCTIONS --------------------%
        function plot_robot(this, color, NN)
            % robot.plot_robot Plots the pose of the robot
            %
            %   PLOT_ROBOT(color, NN) Plots the actual pose of the 
            %   robot in the current figure
            %
            % INPUTS
            %   color: Color of the robot
            %      NN: Number of points of the circle
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. SET DEFAULT COLOR
            if(nargin < 2)
                color = 'b';
            end
            %  1.2. SET DEFAULT NUMBER OF POINTS TO PLOT THE ROBOT
            if(nargin < 3)
                NN = 24;
            end
            %  1.3. MAKE SURE "B" IS DEFINED
            if(isempty(this.B))
                error('robot.plot_robot: Parameter B is not defined.')
            end
            %  1.4. MAKE SURE "x" IS DEFINED
            if(isempty(this.x))
                error('robot.plot_robot: The pose is not defined') 
            end
            %  1.5. GET THE PARAMETER 'B'
            BB = this.B;
                        
            %-----------------------------------------------------------%
            % 2. GET THE POSE OF THE ROBOT
            %-----------------------------------------------------------%
            %  2.1. GET THE LOCATION
            xx = this.x(1);
            yy = this.x(2);
            %  2.2. GET THE ORIENTATION
            theta = this.x(3);
            if(theta > pi ||theta <= -pi)
                error('robot.plot_robot: The angle is not in the range.')
            end
            
            %-----------------------------------------------------------%
            % 3. PLOT THE ROBOT
            %-----------------------------------------------------------%
            %  3.1. COMPUTE THE POINTS OF THE CIRCLE
            ang = linspace(0,2*pi,NN)';
            x_circle = xx*ones(NN,1) + BB*cos(ang);     % x-components
            y_circle = yy*ones(NN,1) + BB*sin(ang);     % y_components
            %  3.2. PLOT THE CIRCLE
            plot(x_circle, y_circle, color,'lineWidth',2)
            hold on
            %  3.3. PLOT THE LINE THAT REPRESENT THE ORIENTATION
            xf = xx + BB*cos(theta);
            yf = yy + BB*sin(theta);
            plot([xx  xf], [yy  yf], color,'lineWidth',3)
        end
        
        function plot_robot3(this, z, B, color, NN)
            % ROBOT.PLOT_ROBOT3 Plots the pose of the robot
            %
            %   PLOT_ROBOT(z, color, NN) Plots the actual pose of the robot
            %   the robot in the current figure
            %
            % INPUTS
            %       z: Value of Z axis
            %   color: Color of the robot
            %      NN: Number of points of the circle
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin<3)
                error('ROBOT.PLOT_ROBOT3: You must pass 3 arguments' )
            end
            %  1.2. SET DEFAULT COLOR
            if(nargin < 4)
                color = 'b';
            end
            %  1.3. SET DEFAULT NUMBER OF POINTS TO PLOT THE ROBOT
            if(nargin < 5)
                NN = 24;
            end
            %  1.4. MAKE SURE "x" IS DEFINED
            if(isempty(this.x))
                error('ROBOT.PLOT_ROBOT3: The pose is undefined') 
            end
            %  1.5. GET THE PARAMETER 'B'
            BB = B;
            
            %-----------------------------------------------------------%
            % 2. GET THE POSE OF THE ROBOT
            %-----------------------------------------------------------%
            %  2.1. GET THE LOCATION
            xx = this.x(1); 
            yy = this.x(2);
            %  2.2. GET THE ORIENTATION
            theta = this.x(3);
            if(theta > pi ||theta <= -pi)
                error('ROBOT.PLOT_ROBOT3: The angle is not in the range.')
            end
            
            %-----------------------------------------------------------%
            % 3. PLOT THE ROBOT
            %-----------------------------------------------------------%
            %  3.1. COMPUTE THE POINTS OF THE CIRCLE
            ang = linspace(0,2*pi,NN)';
            x_circle = xx*ones(NN,1) + BB*cos(ang);     % x-components
            y_circle = yy*ones(NN,1) + BB*sin(ang);     % y_components
           	z_circle = z*ones(NN,1);
            %  3.2. PLOT THE CIRCLE
            plot3(x_circle, y_circle, z_circle, color,'lineWidth',2)
            hold on
            %  3.3. PLOT THE LINE THAT REPRESENT THE ORIENTATION
            xf = xx + BB*cos(theta);
            yf = yy + BB*sin(theta);
            plot3([xx  xf], [yy  yf], [z z], color, 'lineWidth',3)
        end
        
        function display_robot(this, h_robot, color, NN)
            % robot.display_robot Plots the pose of the robot
            %
            %   DISPLAY_ROBOT(H_ROBOT, COLOR, NN) Plots the actual pose
            %   the robot using handles
            %
            % INPUTS
            %  h_robot: Handle to plot the robot
            %    color: Color of the robot
            %       NN: Number of points of the circle
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 2)
                str = ['robot.display_robot:',...
                    'Insufficient number of arguments'];
                error(str)
            end
            %  1.2. CHECK DIMENSION OF THE HANDLE
            if(length(h_robot) ~= 2)
                str = ['robot.display_robot:',...
                    'Handles must be of length 2'];
                error(str)
            end
            %  1.3. SET DEFAULT COLOR
            if(nargin < 3)
                color = 'b';
            end
            %  1.4. SET DEFAULT NUMBER OF POINTS TO PLOT THE ROBOT
            if(nargin < 4)
                NN = 24;
            end
            
            %-----------------------------------------------------------%
            % 2. CHECK AND GET NECCESARY PROPERTIES
            %-----------------------------------------------------------%
            %  2.1. MAKE SURE "B" IS DEFINED
            if(isempty(this.B))
                error('robot.display_robot: Parameter B is not defined.')
            end
            %  2.2. MAKE SURE "x" IS DEFINED
            if(isempty(this.x))
                error('robot.display_robot: Pose is not defined.')
            end
            %  2.3. GET PROPERTIES (TO SPEED UP)
            BB = this.B;
            
            %-----------------------------------------------------------%
            % 3. GET THE POSE OF THE ROBOT
            %-----------------------------------------------------------%            
            %  3.1. GET THE LOCATION
            xx = this.x(1);
            yy = this.x(2);
            %  3.2. GET THE ORIENTATION
            theta = this.x(3);
            if(theta > pi ||theta <= -pi)
                str = ['robot.display_robot:',...
                    'The angle theta is not in the range.'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 4. COMPUTE REPRESENTATIVE POINTS OF THE ROBOT
            %-----------------------------------------------------------%
            %  4.1. COMPUTE THE POINTS OF THE CIRCLE
            ang = linspace(0,2*pi,NN)';
            x_circle = xx*ones(NN,1) + BB*cos(ang);     % x-components
            y_circle = yy*ones(NN,1) + BB*sin(ang);     % y_components
            %  4.2. COMPUTE THE POINTS OF THE ORIENTATION
            xf = xx + BB*cos(theta);
            yf = yy + BB*sin(theta);
            linex = [xx  xf];
            liney = [yy  yf];
                     
            %-----------------------------------------------------------%
            % 5. DRAW THE NEW POSE USING THE HANDLES
            %-----------------------------------------------------------%
            set(h_robot(1),'xdata', x_circle,'ydata',y_circle,'Color',color);
            set(h_robot(2),'xdata', linex,'ydata',liney,'Color',color);
        end
            
        function display_vehicle(this, handle_car)
            % robot.display_vehicle Draws our real robot
            %
            %   DISPLAY_VEHICLE Display the real robot in its actual pose
            %   using the specified handles.
            %
            % INPUTS
            %   handle_car: Handles to fill 2D polygons and draw lines
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS CHECKING
            %-----------------------------------------------------------%    
            %  1.1. MAKE SURE THE POSE IS DEFINED
            if(isempty(this.x))
                error('robot.display_vehicle: The pose is not defined') 
            end
            %  1.2. CHECK TYPE OF ROBOT
            if(isempty(this.NAME_ROBOT))
                str = ['robot.display_vehicle:',...
                    'The robot name is not defined.'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. DISPLAY ACCORDING TO THE ROBOT NAME
            %-----------------------------------------------------------%
            switch(this.NAME_ROBOT)
                case {'R2D2-00', 'R2D2-01'}
                    this.display_R2D2(handle_car);
                case {'R2D2-R1', 'R2D2-R2'}
                    this.display_R2D2_RX(handle_car);
                otherwise
                    error('robot.display_vehicle: Undefined robot name')
            end
        end   
        %----------------- END PLOTTING  FUNCTIONS -------------------%
        
        
        %------------------ STORAGE FUNCTIONS -----------------------%
        function initialize_store(this, CHUNK)
            % robot.initialize_store Save navigation data
            %
            %   INITIALIZE_STORE(CHUNCK) Initialize the structure DATA in 
            %   order to save all the data
            
            %-----------------------------------------------------------%
            % 1. INITIALIZE THE STRUCTURE "DATA"       
            %-----------------------------------------------------------%          
            %  1.1. SET THE INDEX
            DATA.i = 1;
            %  1.2. SAVE POSE "x"
            DATA.X      = zeros(3,CHUNK);
            DATA.X(:,1) = this.x;
            %  1.3. SAVE ODOMETRY "x_ODOMETRY"
            if(isempty(this.x_ODOMETRY)==0)
                DATA.X_ODOMETRY      = zeros(3,CHUNK);
                DATA.X_ODOMETRY(:,1) = this.x_ODOMETRY;
            end
            
            %-----------------------------------------------------------%
            % 2. SAVE THE STRUCTURE "DATA"       
            %-----------------------------------------------------------%  
            this.data         = DATA;
            this.CHUNK_DATA   = CHUNK;
            this.FLAG_STORAGE = 1;
        end
        
        function store_new_data(this)
            % robot.store_new_data Store new data
            %
            %   STORE_NEW_DATA Store the new data of the robot
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. CHECK FLAG
            if(this.FLAG_STORAGE==0)
                str = ['robot.store_new_data:',...
                    'You need to initialize the storage'];
                error(str)
            end
            %  1.2. GET PROPERTIES
            CHUNK = this.CHUNK_DATA;
            %  1.3. ALLOCATE MORE SPACE IS NECCESARY
            if(rem(this.data.i, CHUNK) == 0)
                % Get struct of data
                DATA = this.data;
                % Allocate more space
                DATA.X = [DATA.X  zeros(3,CHUNK)];
                if(isempty(this.x_ODOMETRY)==0)
                    DATA.X_ODOMETRY = [DATA.X_ODOMETRY  zeros(3,CHUNK)];
                end
                % Save new struct of data
                this.data = DATA;
            end
            
            %-----------------------------------------------------------%
            % 2. SAVE DATA
            %-----------------------------------------------------------%
            %  2.1. UPDATE THE INDEX
            i = this.data.i + 1;
            %  2.2. SAVE DATA
            this.data.i      = i;
            this.data.X(:,i) = this.x;
            %  2.3. SAVE ODOMETRY IF IT EXIST
            if(isempty(this.x_ODOMETRY)==0)
                this.data.X_ODOMETRY(:,i) = this.x_ODOMETRY;
            end
        end
        %---------------- END STORAGE FUNCTIONS ---------------------%
        
        
        %----------------- ENCODERS FUNCTIONS  ----------------------%        
        function [nrev1, nrev2] = simulate_encoders(this, u, delta)
            % robot.simulate_encoders Simulate encoders from velocities
            %
            %   [P1,P2] = SIMULATE_ENCODERS(U,DELTA) Compute the lecture 
            %             of the encoders from the velocity controls
            %
            % OUTPUTS
            %   nrev1: Number of revolutions of wheel 1. Right one
            %   nrev2: Number of revolutions of wheel 2. Left one
            %
            % INPUTS
            %       u: Comanded control "u = [v;w]"
            %   delta: Interval of time
                        
            %-----------------------------------------------------------%
            % 1. PARAMETERS CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK DIMENSIONS OF VECTOR OF CONTROLS
            if(length(u)~=2 || size(u,2)~=1)
                error('robot.simulate_encoders: "u" must be of size 2x1') 
            end    
            
            %-----------------------------------------------------------%
            % 2. GET PROPERTIES
            %-----------------------------------------------------------%
            BB = this.B;
            DD = this.D;
            
            %-----------------------------------------------------------%
            % 3. GET THE VELOCITY CONTROLS
            %-----------------------------------------------------------%
            v = u(1);
            w = u(2);
            
            %-----------------------------------------------------------%
            % 4. COMPUTE THE ARC ROTATED BY EACH WHEEL
            %-----------------------------------------------------------%
            if(abs(w)>=1e-3 && abs(v)>=1e-3) % TRANSLATIONAL + ROTATIONAL
                if(w>0) % COUNTER CLOCKWISE
                    % a. Compute the radius
                    R = v/w;        % Of the center
                    R1 = R + BB;    % Of point 1. Right wheel
                    R2 = R - BB;    % Of point 2. Left wheel
                    % b. Compute the rotated angle
                    ALPHA = w*delta;
                    % c. Compute the arc rotated by each wheel.
                    L1 = R1*ALPHA;
                    L2 = R2*ALPHA;
                else % CLOCKWISE
                    % a. Compute the radious
                    R = v/abs(w);
                    R1 = R - BB;        % Of point 1. Right wheel
                    R2 = R + BB;        % Of point 2. Left wheel
                    % b. Compute the rotated angle
                    ALPHA = abs(w*delta);
                    % c. Compute the arc rotated by each wheel.
                    L1 = R1*ALPHA;
                    L2 = R2*ALPHA;
                end
                
            elseif(abs(w)>1e-3 && abs(v)<1e-3) % ROTATIONAL
                % a. Compute the rotated angle around its center
                alpha_a = w*delta;
                % b. Compute the arc rotated by each wheel.
                L1 = alpha_a*BB; 
                L2 = -L1;
                
            elseif(abs(w)<1e-3 && abs(v)>1e-3) % TRANSLATIONAL
                % a. Distance traveled
                d = v*delta;
                % b. Compute the arc rotated by each wheel
                L1 = d;
                L2 = d; 
                
            elseif(abs(w)<1e-3 && abs(v)<1e-3) % NO MOTION
                % The rotational and translational velocities are 
                % almost zero
                L1 = 0;
                L2 = 0;
            end
            
            %-----------------------------------------------------------%
            % 5. COMPUTE THE NUMBER OF PULSES
            %-----------------------------------------------------------%
            nrev1 = L1/(pi*DD);
            nrev2 = L2/(pi*DD);           
        end
            
        function [xp_ODOMETRY, alpha, R, sign_w, mark] = ...
                compute_pose_from_encoder(robot, nrev1, nrev2, m)
            % robot.compute_pose_from_encoder Compute the new pose
            %                                 using the encoders
            %
            %   COMPUTE_POSE_FROM_ENCODER This method allow us to compute 
            %   the next pose of the robot accoding to the number of
            %   revolutions of the wheels.
            %
            % NOTES: 
            %   - This method updates the 'internal' odometry pose
            %   - This method also computes the the odometry controls 
            %     "robot.u_ODOMETRY"
            %
            % OUTPUTS
            %  xp_ODOM: Final "internal odometry" pose
            %    alpha: Has the following interpretations
            %              - If the movement is a pure rotacion around its
            %                center, it gives the angle of rotacion around 
            %                its center
            %              - If the movement is a translation, it gives the
            %                distance traveled by the robot
            %              - If the movement is a "trans + rot", it gives 
            %                the angle of rotacion around the center of the 
            %                circle
            %        R: Has the following interpretations
            %              - If the movement is a pure rotacion around its 
            %                center, it gives the number of full rotations 
            %                around its center
            %              - If the movement is a translation, it takes "Inf"
            %              - If the movement is a "trans + rot", it gives 
            %                the radious of the circle on which it rotates
            %   sign_w:  Returns the sign of the rotational velocity
            %              - "+1": COUNTER-CLOCK WISE
            %              - "-1": CLOCKWISE
            %              -  "0": w is zero
            %     mark:  Has the following values:
            %              - 0: If the movement is a pure rotacion
            %              - 1: If the movement is a translation
            %              - 2: If the movement is a trans + rotation
            %
            % INPUTS
            %   nrev1: Number of revolutions of wheel 1. Right one
            %   nrev2: Number of revolutions of wheel 2. Left one
            %       m: Turn on/off messages
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 3)
                str = ['robot.compute_pose_from_encoder:',...
                    'Insuficient number of arguments'];
                error(str)
            end
            %  1.2. SET DEFAULT VALUE FOR "m"
            if(nargin < 4)
                m = 0;
            end
            %  1.3. MAKE SURE "D" IS DEFINED
            if(isempty(robot.D))
                str = ['robot.compute_pose_from_encoder:',...
                    'D is not defined'];
                error(str)
            end
            %  1.4. MAKE SURE "x_ODOMETRY" IS DEFINED
            if(isempty(robot.x_ODOMETRY))
                str = ['robot.compute_pose_from_encoder:',...
                    '"x_ODOMETRY" is not defined'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. GET CURRENT ODOMETRY POSE
            %-----------------------------------------------------------%          
            x_odometry = robot.x_ODOMETRY;
            
            %-----------------------------------------------------------%
            % 3. COMPUTE NEXT ODOMETRY LOCAL POSE
            %-----------------------------------------------------------%
            [xp_local, alpha, R, mark] = ...
                robot.compute_local_pose_from_encoder(nrev1, nrev2, m);
            sign_w = sign(alpha);
            
            %-----------------------------------------------------------%
            % 4. CONVERT TO ODOMETRY GLOBAL FRAME
            %-----------------------------------------------------------%
            frame            = x_odometry;
            xp_ODOMETRY      = local_to_global(xp_local, frame);
            robot.x_ODOMETRY = xp_ODOMETRY;  % save in robot
            
            %-----------------------------------------------------------%
            % 5.COMPUTE THE VECTOR OF DELTAS
            %-----------------------------------------------------------%
            u = compute_odometry_controls(x_odometry, xp_ODOMETRY, m);
            robot.u_ODOMETRY = u;
        end
        
        function [xp_local, alpha, R, mark] = ...
                compute_local_pose_from_encoder(robot, nrev1, nrev2, m)
            % robot.compute_local_pose_from_encoder Compute the new "local"
            %                                       pose using the encoders
            %
            %   [XP,...] = COMPUTE_LOCAL_POSE_FROM_ENCODER(NREV1,NREV2,M)
            %              This is to compute the new "local" pose of the  
            %              robot using the number of revolutions of the 
            %              wheels. (We use the local frame of the 
            %              robot to describe xp)
            %
            % OUTPUTS
            %   xp_local: Final "local" pose predicted by the encoders
            %      alpha: Has the following interpretations
            %              - If the movement is a pure rotacion around its 
            %                center, it gives the angle of rotacion around 
            %                its center
            %              - If the movement is a translation, it gives the
            %                distance traveled by the robot
            %              - If the movement is a "trans + rot", it gives  
            %                the angle of rotacion around the center of 
            %                the circle
            %         R: Has the following interpretations
            %              - If the movement is a pure rotacion around its 
            %                center, it gives the number of full rotations 
            %                around its center
            %              - If the movement is a translation, it takes "Inf"
            %              - If the movement is a "trans + rot", it gives 
            %                the radious of the circle on which it rotates
            %     mark:  Has the following values:
            %               - 0: If the movement is a pure rotacion around its center
            %               - 1: If the movement is a translation
            %               - 2: If the movement is a trans + rotation
            %
            % INPUTS
            %   nrev1: Number of pulses from the wheel 1. Right one
            %   nrev2: Number of pulses from the wheel 2. Left one
            %       m: Turn on/off messages
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS' CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NECCESARY ARGUMENTS
            if(nargin < 3)
                str = ['robot.compute_local_pose_from_encoder:',...
                    'Insuficient number of arguments'];
                error(str)
            end
            %  1.2. SET DEFAULT VALUE OF "m"
            if(nargin < 4)
                m = 0;
            end
            
            %-----------------------------------------------------------%
            % 2. GET THE MAIN DIMENSIONS OF THE ROBOT
            %-----------------------------------------------------------%
            DD = robot.D;
            BB = robot.B;
            
            %-----------------------------------------------------------%
            % 3. MAIN FUNCTION. According to the type of motion(pure rotation,
            %    pure translation, or trans+rot) we compute the outputs
            %-----------------------------------------------------------%
            if(abs(nrev1+nrev2)<1e-4)  % PURE ROTATION
                %  - Note that we allow some degree of error between the 
                %    number of revolutions
                % A. SET FLAG AND DISPLAY MESSAGES IF DESIRED
                mark = 0;
                if(m == 1)
                    str = ['robot.COMPUTE_LOCAL_POSE_FROM_ENCODER:',...
                        'The movement is a pure rotation around its center'];
                    disp(str)
                end
                % B. COMPUTE THE ARC ROTATED BY THE WHEELS
                L1 = pi*DD*nrev1;       % Arc rotated by wheel 1
                L2 = pi*DD*nrev2;       % Arc rotated by wheel 2
                arc = (L1-L2)/2;        % Mean
                % C. ANGLE OF ROTATION AROUND ITS CENTER
                alpha = arc/BB;
                % D. COMPUTE THE NEW POSE
                %   -> Set position
                xx_p = 0;
                yy_p = 0;
                %   -> Set the new angle
                theta_p = alpha;
                theta_p = pi_to_pi(theta_p);
                %   -> Set the new local pose
                xp_local = [xx_p; yy_p; theta_p];
                % E. SET NUMBER OF FULL ROTATIONS AROUND ITS CENTER
                R = alpha/(2*pi);
                
            elseif(abs(nrev1-nrev2)<1e-4) % PURE TRANSLATION
                %  - Note that we allow some degree of error between the 
                %    number of revolutions
                % A. SET FLAG AND DISPLAY MESSAGES IF DESIRED
                mark = 1;
                if(m == 1)
                    str = ['  robot.COMPUTE_LOCAL_POSE_FROM_ENCODER:',...
                        'The movement is a translation'];
                    disp(str)
                end
                % B. COMPUTE THE ARC ROTATED BY THE WHEELS
                L1 = pi*DD*nrev1;       % Arc rotated by wheel 1
                L2 = pi*DD*nrev2;       % Arc rotated by wheel 2
                % C. DISTANCE OF THE ROBOT
                d = (L1 + L2)/2;
                % D. COMPUTE THE NEW POSE
                %   -> Set position
                xx_p = d;
                yy_p = 0;
                %   -> Set orientation
                theta_p = 0;
                xp_local = [xx_p; yy_p; theta_p];
                % E. SET OTHER VARIABLES
                alpha = d;      % Distance traveled
                R = inf;
                
            else  % TRANSLATIONAL + ROTATIONAL
                % A. SET FLAG AND DISPLAY MESSAGES IF DESIRED
                mark = 2;
                if(m == 1)
                    str = ['  robot.COMPUTE_LOCAL_POSE_FROM_ENCODER:',...
                        'The movement is a rotation + translation'];
                    disp(str)
                end
                % B. COMPUTE THE ARC ROTATED BY THE WHEELS
                L1 = pi*DD*nrev1;
                L2 = pi*DD*nrev2;
                % C. COMPUTE THE ANGLE OF ROTATION AROUND THE CIRCLE
                alpha = (L1 - L2)/(2*BB);
                % D. COMPUTE THE NEW POSE
                if(alpha > 0) % COUNTER CLOCKWISE
                    % Compute the radious of the circle for the center point 
                    R = L2/alpha + BB;
                    % Compute the new pose
                    %   -> Set position
                    xx_p = R*sin(alpha);
                    yy_p = R*(1-cos(alpha));    
                    %   -> Set orientation
                    theta_p = alpha;
                    theta_p = pi_to_pi(theta_p);
                else % CLOCKWISE
                    % Compute the radious of the circle for the center point 
                    R = L1/(abs(alpha)) + BB;
                    % Compute the new pose
                    %   -> Set position
                    xx_p =  R*sin(-alpha);
                    yy_p = -R*(1-cos(alpha));
                    %   -> Set orientation
                    theta_p = alpha;
                    theta_p = pi_to_pi(theta_p);                  
                end
                % D. SET THE NEW POSE
                xp_local = [xx_p; yy_p; theta_p];
            end
        end
        %--------------- END ENCODERS FUNCTIONS  --------------------%
        
        
        %--------------- VELOCITY MOTION FUNCTIONS ------------------%      
        function set_velocity_noise_parameters(this, alpha)
            % robot.set_velocity_noise_parameters Set motion error parameters
            %                                     for the velocity model
            %                           
            %   SET_VELOCITY_NOISE_PARAMETERS(ALPHA) This is to set
            %   robot-specific motion error parameters for the velocity
            %   motion model of a robot
            %
            % INPUTS
            %   alpha: Vector of motion error parameters
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK NUMBER OF ARGUMENTS
            if(nargin < 2)
                str = ['robot.set_velocity_noise_parameters:',...
                    'Insufficient number of arguments'];
                error(str)
            end
            %  1.2. CHECK DIMENSION OF ALPHA
            if(length(alpha)~=6)
                str = ['robot.set_velocity_noise_parameters:',...
                    'alpha must be of size 6'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. SET THE NEW VECTOR OF NOISE
            %-----------------------------------------------------------%
            this.alpha_VELOCITY = alpha;
            this.LEVEL_NOISE    = -1;         % Reset this flag
        end
        
        function [xp, xc, yc, r, f] = ...
                noise_free_motion_model_velocity(robot,u,delta)
            % robot.noise_free_motion_model  Noise-free motion of the robot
            %
            %   NOISE_FREE_MOTION_MODEL Implements the algorithm that 
            %   describe the exact motion of a differential-drive mobile
            %   robot. See Page 127. EQ(5.9)
            %
            % OUTPUTS
            %     xp: The new pose
            %  xc,yc: Center of the circle of rotation (Inf in trans. motion) 
            %      r: Radious of the circle of rotation (Inf in trans. motion) 
            %      f: Flag (1:Rot+trans motion,  0:Pure trans. motion)
            %
            % INPUTS 
            %      u: Vector of controls [v; w];
            %  delta: Time interval(seg.)
            %
            %  NOTES:
            %   - The internal pose "x" of the robot is updated 
            %     with the new pose
                     
            %-----------------------------------------------------------%
            % 1. CHECK NECCESARY ARGUMENTS
            %-----------------------------------------------------------%
            %  1.1. MAKE SURE THAT THE POSE OF THE ROBOT IS DEFINED
            if(isempty(robot.x))
                str = ['robot.noise_free_motion_model:',...
                    'The pose is not defined'];
                error(str) 
            end
            
            %-----------------------------------------------------------%
            % 2. GET THE COMPONENTS OF THE PREVIOUS POSE AND THE CONTROLS
            %-----------------------------------------------------------%
            %  2.1. GET THE COMPONENTS OF THE POSE AT TIME "t-1"
            xx = robot.x(1);
            yy = robot.x(2);
            theta = robot.x(3);
            if(theta > pi ||theta <= -pi)
                str = ['robot.noise_free_motion_model:',...
                    'The angle is not in the range'];
                error(str)
            end
            %  2.2. GET THE VELOCITIES FROM THE CONTROL "u"
            v = u(1);
            w = u(2);
            
            %-----------------------------------------------------------%
            % 3. COMPUTE THE NEW POSE
            %-----------------------------------------------------------%
            % There are two cases:
            %   - The robot performs a translational + rotational velocity, 
            %     so the robot does a turn around a circle.
            %   - The robot performs only a translational velocity, which
            %     corresponds to a straight line
            %
            if(abs(w)>1e-6)  % IF THE ROTATIONAL IS NOT ALMOST ZERO
                % 3.1. Compute the coordinates of the center of the circle
                xc = xx - (v/w)*sin(theta);       % EQ(5.7)
                yc = yy + (v/w)*cos(theta);       % EQ(5.8)
                % 3.2. Compute the new pose: [xx_p, yy_p, theta_p]. EQ(5.9)
                xx_p = xc + (v/w)*sin(theta + w*delta);
                yy_p = yc - (v/w)*cos(theta + w*delta);
                theta_p = theta + w*delta;
                % 3.3. Set radious of the circle and the flag
                r = abs(v/w);
                f = 1;
            else  % IF THE ROTATIONAL VELOCITY IS ALMOST ZERO
                % 2.1. Distance traveled
                d = v*delta;
                % 2.2. Compute the new pose
                xx_p = xx + d*cos(theta);
                yy_p = yy + d*sin(theta);
                theta_p = theta;
                % 2.3. Set the coordinates of the center of the circle
                xc = Inf;
                yc = Inf;
                % 2.4. Set radious of the circle and the flag
                r = Inf;
                f = 0;
            end
            
            %-----------------------------------------------------------%
            % 4. SET THE NEW POSE
            %-----------------------------------------------------------%
            %  4.1. MAKE SURE THAT THE NEW "theta" IS IN THE RANGE [-pi,pi] 
            theta_p = pi_to_pi(theta_p);
            %  4.2. RETURN  THE NEW POSE
            xp = [xx_p; yy_p; theta_p];
            %  4.3. UPDATE THE POSE OF THE ROBOT
            robot.x = xp;
        end 
        
        function xp = sample_motion_model_velocity(this, u, dt)
            % robot.sample_motion_model_velocity  Sample a new pose of
            %                                     the robot.
            %
            %   SAMPLE_MOTION_MODEL_VELOCITY(u, delta) Implements the 
            %   algorithm that is described in the table 5.3. Page 124
            % 
            % OUTPUT
            %      xp: Sample pose at time "t" of the robot.
            %
            % INPUTS
            %    u: Vector of control [v;w]
            %   dt: Delta of time      
            %
            %  NOTE:
            %   - The internal pose "x" of the robot is updated 
            %     with the new pose
            
            %-----------------------------------------------------------%
            % 1. PARAMETERS CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK THE ANGLE
            theta = this.x(3);
            if(theta > pi || theta <= -pi)
                str = ['robot.sample_motion_model_velocity:',...
                    'The angle theta of the robot is not in the range.'];
                error(str)
            end
            alpha = this.alpha_VELOCITY;
            
            %-----------------------------------------------------------%
            % 2. COMPUTE RESULT IN THE CASE OF GAUSSIAN MODEL
            %-----------------------------------------------------------%
            if(strcmp(this.MOTION_MODEL, 'GAUSSIAN')==1)
                % COMPUTE GAUSSIAN PARAMETERS
                [mu, R] = this.compute_gaussian_model_velocity(u,dt);
                % TAKE 1 SAMPLE
                N = 1;
                SAMPLES      = mvnrnd(mu,R,N);
                SAMPLES(:,3) = pi_to_pi(SAMPLES(:,3));
                % SET SAMPLE
                xp = SAMPLES(:)';
                % SAVE NEW POSE
                this.x = xp; 
                return
            end
            
            %-----------------------------------------------------------%
            % 3. COMPUTE ROTATIONAL MOTION IN THE IMPROVED MODEL
            %-----------------------------------------------------------%
            if(abs(u(1))<=1e-4 && strcmp(this.MOTION_MODEL, 'IMPROVED'))
                w = u(2);
                % GAUSSIAN
                mu = [0;0;w*dt];
                R = diag( [ alpha(2)*(w^2)*dt/1.1412;...
                            alpha(2)*(w^2)*dt/1.4142;...
                           (alpha(4) + alpha(6))*(w^2)*dt] );
                % TAKE 1 SAMPLE
                NN = 1;
                samples = mvnrnd(mu,R,NN)';
                xp      = this.x + samples;
                xp(3)   = pi_to_pi(xp(3));
                % SAVE NEW POSE
                this.x = xp; 
                return;
            end

            %-----------------------------------------------------------%
            % 4. SET TRUE VELOCITIES IN THE CASE OF THE OTHER MODELS
            %-----------------------------------------------------------%
            switch(this.MOTION_MODEL)
                case 'STANDARD'
                    %  3.1. Translational velocity
                    var = alpha(1)*u(1)^2 + alpha(2)*u(2)^2;  % Variance
                    v   = u(1) + sqrt(var)*randn(1);
                    %  3.2. Rotational velocity
                    var = alpha(3)*u(1)^2 + alpha(4)*u(2)^2;  % Variance
                    w   = u(2) + sqrt(var)*randn(1);
                    %  3.3. Additional rotational velocity
                    var   = alpha(5)*u(1)^2 + alpha(6)*u(2)^2;  % Variance
                    gamma = sqrt(var)*randn(1);
                case {'STANDARD2', 'IMPROVED'}
                    %  3.1. Translational velocity
                    var = 1/dt*(alpha(1)*u(1)^2  +  alpha(2)*u(2)^2);
                    v   = u(1) + sqrt(var)*randn(1);
                    %  3.2. Rotational velocity
                    var = 1/dt*(alpha(3)*u(1)^2  +  alpha(4)*u(2)^2);
                    w   = u(2) + sqrt(var)*randn(1);
                    %  3.3. Additional rotational velocity
                    var   = 1/dt*(alpha(5)*u(1)^2  +  alpha(6)*u(2)^2);
                    gamma = sqrt(var)*randn(1);
            end
            
            %-----------------------------------------------------------%
            % 5. COMPUTE A SAMPLE OF THE NEW POSE
            %-----------------------------------------------------------%
            xx = this.x(1);
            yy = this.x(2);
            if(abs(w)>1e-5)
                xx_p    = xx - (v/w)*sin(theta) + (v/w)*sin(theta + w*dt);
                yy_p    = yy + (v/w)*cos(theta) - (v/w)*cos(theta + w*dt);
                theta_p = theta + w*dt+ gamma*dt;   
            else % TRANSLATIONAL MOTION 
                % Distance traveled
                d = v*dt;
                % New pose
                xx_p = xx + d*cos(theta);
                yy_p = yy + d*sin(theta);
                theta_p = theta + gamma*dt;               
            end
            
            %-----------------------------------------------------------%
            % 6. SET THE NEW POSE
            %-----------------------------------------------------------%
            %  6.1. CHECK THE NEW ANGLE (-pi,pi] 
            theta_p = pi_to_pi(theta_p);
            %  6.2. RETURN THE NEW POSE
            xp = [xx_p; yy_p; theta_p];
            %  6.3. SET THE NEW POSE OF THE ROBOT
            this.x = xp;   
        end
        
        function [p, v_true, w_true, gamma, mu, xc, yc, r, delta_theta] = ...
                motion_model_velocity(this, xp, u, dt, m)
            % robot.motion_model_velocity  Compute probability "p(xp|u,x)"
            %                                     
            %   MOTION_MODEL_VELOCITY Implements the algorithm "motion_
            %   model_velocity" that is described in the table 5.1. Page 123.
            % 
            % OUTPUTS
            %        p: p(xp|u,x) probability of being at "xp" after
            %           executing  control "u" beginning in state "x", 
            %           assuming that the control is carried for a fixed
            %           duration "delta".
            %   v_true: The true trans. velocity for the robot to reach "xp"
            %   w_true: The true rot. velocity for the robot to reach "xp"
            %    gamma: Additional rotational velocity to achieve the final heading
            %       mu: Distance from the center circle to the half-way
            %            point between "x" and "xp"
            %   xc, yc: Coordinates of the center of the circle
            %        r: Radius of the circle 
            %  delta_theta: Change in heading direction
            %
            % INPUTS
            %    xp: Vector that represent the proposed pose "xp = [x;y;theta]" 
            %        at time "t" 
            %     u: Vector of control = [v; w]; 
            %    dt: Time interval(seg.)
            %     m: Turn on/off some messages
            
            %-----------------------------------------------------------%
            % 1. CHECK AND GET SOME PROPERTIES
            %-----------------------------------------------------------%
            %  1.1. MAKE SURE "x(t-1)" IS DEFINED
            if(isempty(this.x))
                error('robot.motion_model_velocity: The pose is not defined')
            end
            %  1.2. CHECK THE ANGLES
            theta = this.x(3);
            if(theta > pi || theta <= -pi)
                str = ['robot.motion_model_velocity:',...
                    'The angle theta of the robot is not in the range.'];
                error(str)
            end
            %  1.3. BY DEFAULT WE DON'T SHOW MESSAGES
            if(nargin < 5)
                m = 0;
            end
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1. GET THE COMPONENTS OF THE POSE "x(t-1)"
            xx    = this.x(1);
            yy    = this.x(2);
            theta = this.x(3);
            %  2.2. GET THE COMPONENTS OF THE POSE "x(t)"
            xx_p    = xp(1);
            yy_p    = xp(2);
            theta_p = xp(3);
            %  2.3. GET THE VELOCITIES
            v = u(1);
            w = u(2);
            %  2.4. COMPUTE "mu": Distance from the center circle to the
            %       half-way point between "x" and "xp"
            num = (xx - xx_p)*cos(theta) + (yy - yy_p)*sin(theta);
            den = (yy - yy_p)*cos(theta) - (xx - xx_p)*sin(theta);
            mu = 0.5*num./den;
            
            %-----------------------------------------------------------%
            % 3. COMPUTE ROTATIONAL MOTION IN THE IMPROVED MODEL
            %-----------------------------------------------------------%
            if(abs(u(1))<=1e-4 && strcmp(this.MOTION_MODEL, 'IMPROVED'))
                alpha = this.alpha_VELOCITY;
                % SET GAUSSIAN
                muG    = this.x + [0;0;w*dt];
                muG(3) = pi_to_pi(muG(3));
                R = diag( [ alpha(2)*(w^2)*dt/1.1412;...
                            alpha(2)*(w^2)*dt/1.4142;...
                           (alpha(4) + alpha(6))*(w^2)*dt] );
                % COMPUTE DENSITIES
                p = gauss_ND_robot(xp,muG,R);
                % SET OTHER OUTPUTS
                xc = -100; yc=-100; r=-100;
                v_true=-100; w_true=-100; gamma=-100; 
                delta_theta=-100;
                return
            end
            
            %-----------------------------------------------------------%
            % 4. COMPUTE RESULT IN THE CASE OF GAUSSIAN MODEL
            %-----------------------------------------------------------%
            if(strcmp(this.MOTION_MODEL, 'GAUSSIAN')==1)
                % COMPUTE GAUSSIAN PARAMETERS
                [muG, R] = this.compute_gaussian_model_velocity(u,dt);
                % COMPUTE PROBABILITY
                p = gauss_ND_robot(xp,muG,R);
                % SET OTHER OUTPUTS (this is just for debugging)
                %   - Compute the center of rotation
                xc = 0.5*(xx + xx_p) + mu.*(yy - yy_p);
                yc = 0.5*(yy + yy_p) + mu.*(xx_p - xx);
                %   - Compute the radius of the circle
                r = sqrt((xx - xc).^2 + (yy - yc).^2);
                %   - Other outputs
                v_true=-100; w_true=-100; gamma=-100; 
                delta_theta=-100;
                return
            end
            
            %-----------------------------------------------------------%
            % 5. MAIN CODE
            %-----------------------------------------------------------%
            %  5.1. ACCORDING TO THE KIND OF MOTION WE COMPUTE:
            %        -> the true translacional velocity
            %        -> the true rotational velocity
            %        -> the corrected delta_theta
            if(abs(xx - xx_p)==0 && abs(yy - yy_p)==0)
                % A. THE POSITION AT "t-1" AND "t" ARE THE SAME
                %   - Show message if desired
                if(m == 1)
                    disp('  The real motion is to be static or pure ideal rotational')
                end
                %   - Set true velocities
                v_true      = 0;
                delta_theta = pi_to_pi(theta_p - theta);
                w_true      = delta_theta/dt;
                %   - Set other variables
                xc = inf;   yc = inf;
                r = inf;    mu = inf;            
            
            elseif(abs(mu) > 1e5)
                % B. THE RADIUS OF THE CIRCLE IS VERY BIG, SO THE MOTION
                %    IS CONSIDERED TO BE TRANSLATIONAL
                %   - Show message if desired
                if(m == 1)
                    disp('  The real motion for this test point is translational')
                end
                %   - Distance traveled
                d = sqrt((yy_p - yy)^2 + (xx_p - xx)^2);
                %   - Get the angle of the movement 
                theta_t = atan2(yy_p - yy, xx_p - xx);
                %   - Compute the true translational velocities.
                if (sign(theta_t) == sign(theta))
                    v_true = d/dt;       % Forward motion
                else
                    v_true = -d/dt;      % Backward motion
                end
                %   - Set the rotational velocities and change in angle
                w_true = 0;
                delta_theta = 0;
                %   - Set other variables
                xc = inf;   yc = inf;
                r = inf;
                                
            else  % TRANSLATIONAL + ROTATIONAL
                %   - Show message if desired
                if(m==1)
                    disp('  The real motion for this test point is trans + rot')
                end
                % COORDINATES OF THE CENTER
                %     NOTE: In some cases "mu" is negative. This happens
                %           when the test point is more than 180
                xc = 0.5*(xx + xx_p) + mu.*(yy - yy_p);
                yc = 0.5*(yy + yy_p) + mu.*(xx_p - xx);
                % RADIUS OF THE CIRCLE
                r = sqrt((xx - xc).^2 + (yy - yc).^2);
                % CHANGE OF HEADING DIRECTION
                theta_f = atan2(yy_p - yc, xx_p - xc);
                theta_f = pi_to_pi(theta_f);
                theta_o = atan2(yy - yc, xx - xc);
                theta_o = pi_to_pi(theta_o);
                delta_theta = pi_to_pi(theta_f - theta_o);
                % TRUE VELOCITIES
                w_true = delta_theta/dt;
                v_true = (abs(delta_theta)/dt)*r;
                
                % CORRECT VELOCITIES FOR BACKWARD MOTION
                vector = [xx_p-xx; yy_p-yy];
                M = [cos(theta)  sin(theta); -sin(theta)  cos(theta)];
                vector_p = M*vector;
                if(vector_p(1)<0)
                    v_true = -v_true;
                    if(m==1)
                        disp('  ...doing correction')
                    end
                end
            end
            %  5.2. ADDITIONAL ROTATIONAL VELOCITY
            gamma = pi_to_pi(theta_p-theta)/dt - w_true;   
            
            %-----------------------------------------------------------%
            % 6. RETURN THE PROBABILITY "p : p(xp | u, x)"
            %-----------------------------------------------------------%        
            %  6.1. COMPUTE VELOCITY ERRORS
            error_v = v - v_true;
            error_w = w - w_true;
            error_gamma = gamma;
            %  6.2. COMPUTE VARIANCE
            alpha = this.alpha_VELOCITY;
            switch(this.MOTION_MODEL)
                case 'STANDARD'
                    var_error_v     = alpha(1)*v^2 + alpha(2)*w^2;
                    var_error_w     = alpha(3)*v^2 + alpha(4)*w^2;
                    var_error_gamma = alpha(5)*v^2 + alpha(6)*w^2;
                case {'STANDARD2', 'IMPROVED'}
                    var_error_v     = 1/dt*(alpha(1)*v^2 + alpha(2)*w^2);
                    var_error_w     = 1/dt*(alpha(3)*v^2 + alpha(4)*w^2);
                    var_error_gamma = 1/dt*(alpha(5)*v^2 + alpha(6)*w^2);
                otherwise
                    str = ['robot.motion_model_velocity:',...
                    'The motion model is not STANDARD, STANDARD2, IMPROVED.'];
                    error(str)
            end
            %  6.3. COMPUTE PROBABILITY OF THE ERRORS(mean = '0')
            p_error_v     = gauss_1d(0, var_error_v, error_v);
            p_error_w     = gauss_1d(0, var_error_w, error_w);
            p_error_gamma = gauss_1d(0, var_error_gamma, error_gamma);
            %  6.4. RETURN PROBABILITY
            p = p_error_v.*p_error_w.*p_error_gamma;
        end
        
        function [mu, R] = compute_gaussian_model_velocity(this,u,dt)
            % robot.compute_gaussian_model_velocity Gaussian velocity model
            %
            %   COMPUTE_GAUSSIAN_MODEL_VELOCITY(u,DT) Compute the gaussian
            %   velocity model as described in the paper:
            %       "Time-Invariant Gaussian Velocity Motion Models 
            %       for Mobile Robots"
            %
            % OUTPUTS
            %      mu: Mean of the Gaussian motion model
            %       R: Covarianze of the Gaussian motion model
            %
            % INPUTS
            %    u: Vector of controls
            %   dt: Time interval
            
            %----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %----------------------------------------------------------%
            %  1.1. GET THE CONTROLS
            v = u(1);
            w = u(2);
            %  1.2. GET THE ANGLE OF THE CURRENT POSE
            theta = this.x(3);
            %  1.3. GET NOISE PARAMETERS
            alpha = this.alpha_VELOCITY;
            
            %----------------------------------------------------------%
            % 2. COMPUTE THE MEAN OF THE MOTION MODEL
            %     -> We also get in "f" the kind of motion 
            %           f=1  if  abs(w)>1e-6
            %----------------------------------------------------------%
            [mu,~,~,~,f] = noise_free_motion_model_velocity(this.x, u, dt);
            
            %-----------------------------------------------------------%
            % 3. COMPUTE ROTATIONAL MOTION IN PURE ROTATION
            %-----------------------------------------------------------%
            if(abs(u(1))<=1e-4 && 1)
                %mu = [0;0;w*dt];
                R = diag( [ alpha(2)*(w^2)*dt/1.1412;...
                            alpha(2)*(w^2)*dt/1.4142;...
                           (alpha(4) + alpha(6))*(w^2)*dt] );
                return
            end
            
            %----------------------------------------------------------%
            % 4. COMPUTE THE JACOBIAN WRT THE SPACE OF CONTROLS
            %----------------------------------------------------------%
            if(f == 0)  % translational motion. See paper
                % The first column has another term -1/2*w*dt^2*sin(th)
                % but we can eliminate it
                V = [dt*cos(theta)  -1/2*v*dt^2*sin(theta)    0;
                     dt*sin(theta)   1/2*v*dt^2*cos(theta)    0;
                           0                   dt            dt];
            else
                c1 = [(-sin(theta) + sin(theta + w*dt))/w;
                      ( cos(theta) - cos(theta + w*dt))/w;
                                    0];
                c2 = [ v*(sin(theta) - sin(theta+w*dt))/w^2  +  v*cos(theta+w*dt)*dt/w;
                      -v*(cos(theta) - cos(theta+w*dt))/w^2  +  v*sin(theta+w*dt)*dt/w;
                                                            dt];
                c3 = [0;0;dt];
                V = [c1  c2  c3];
            end
            
            %----------------------------------------------------------%
            % 5. COMPUTE THE MATRIX 'R'
            %----------------------------------------------------------%
            %  5.1. COMPUTE THE COVARIANCE MATRIX IN CONTROL SPACE       
            M = 1/dt*diag([alpha(1)*v^2 + alpha(2)*w^2,...
                           alpha(3)*v^2 + alpha(4)*w^2,...
                           alpha(5)*v^2 + alpha(6)*w^2]);
            %  5.2. COMPUTE MATRIX 'R'
            R = V*M*V';
            %  5.3. TO AVOID NUMERICAL ERRORS IN PURE ROTATIONS
            %       IN THE CASE OF PDF COMPUTATION
            %       (Si usamos la aproximacion Gaussiana -ver arriba- esto
            %       ya no se usa)
            if(abs(u(1))<1e-6)
                R(1,1) = R(1,1) + 1e-8;  % Demo_19
            end
        end
        %-------------- END VELOCITY MOTION FUNCTIONS ---------------%
        
        
        %--------------- ODOMETRY MOTION FUNCTIONS ------------------%
        function set_odometry_noise_parameters(robot, alpha)
            % robot.set_odometry_noise_parameters Set motion error parameters
            %                           
            %   SET_VODOMETRY_NOISE_PARAMETERS(ROBOT, ALPHA) This is to set
            %   robot-specific motion error parameters for the odometry
            %   model of a robot
            %
            % INPUTS
            %   alpha: Vector of motion error parameters
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK DIMENSIONS
            if(length(alpha)~=4)
                str = ['robot.set_odometry_noise_parameters:',...
                    'Alpha must be of size 4'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. SET THE NEW VECTOR OF NOISE
            %-----------------------------------------------------------%
            robot.alpha_ODOMETRY = alpha;
            robot.LEVEL_NOISE = -1;         % SET FLAG OF NOISE
        end
        
        function xp = noise_free_motion_model_odometry(robot, u)
            % robot.noise_free_motion_model_odometry Implements the "noise-free" 
            %                                        motion model odometry
            %
            %   XP = NOISE_FREE_MOTION_MODEL_ODOMETRY(U) Computes the 
            %        new pose "xp" of the robot considering a "noise-free" 
            %        motion of the robot. That is, we consider as perfect 
            %        the meassurements of the encoder to predict the next 
            %        pose of the robot
            % 
            % OUTPUTS
            %   xp: Sample pose at time "t" of the robot.
            %
            % INPUTS
            %    u: Vector of controls [drot1; dtrans; drot2] computed from 
            %       the encoders
            
            %-----------------------------------------------------------%
            % 1. PARAMETERS CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK THE DIMENSION OF THE VECTOR OF CONTROLS
            if(length(u)~=3)
                str = ['robot.noise_free_motion_model_odometry:',...
                    '"u" must be of size 3x1'];
                error(str);
            end
            %  1.2. CHECK EXISTENCE OF THE POSE
            if(isempty(robot.x))
                str = ['robot.noise_free_motion_model_odometry:',...
                    '"x" is not defined'];
                error(str);
            end
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1. GET THE POSE OF THE ROBOT AT TIME "t-1"
            xx = robot.x(1);
            yy = robot.x(2);
            theta = robot.x(3);
            %  2.2. GET THE DELTAS FROM THE ENCODERS
            drot1  = u(1);
            dtrans = u(2);
            drot2  = u(3);
            
            %-----------------------------------------------------------%
            % 3. COMPUTE THE NEW POSE
            %-----------------------------------------------------------%
            %  3.1. COMPUTE THE POSITION
            xx_p = xx  +  dtrans*cos(theta + drot1);
            yy_p = yy  +  dtrans*sin(theta + drot1);
            %  3.2. COMPUTE THE ORIENTATION
            theta_p = theta  +  drot1  +  drot2;
            theta_p = pi_to_pi(theta_p);
            %  3.3. RETURN THE NEW POSE
            xp = [xx_p; yy_p; theta_p];
            %  3.4. SET THE NEW POSE OF THE ROBOT
            robot.x = xp;
        end
        
        function xp = sample_motion_model_odometry(robot, u)
            % robot.sample_motion_model_odometry Sample from the odometry
            %                                    model
            %
            %   XP = SAMPLE_MOTION_MODEL_ODOMETRY(U,X) Implements the
            %        algorithm "sample_motion_model_odometry" that is
            %        described in the Thruns' book , and also the models 
            %        described in the papers:
            % 
            % OUTPUTS
            %   xp: Sample pose at time "t" of the robot.
            %
            % INPUTS
            %   u: Vector of control [drot1; dtrans; drot2] from the encoders
            
            %-----------------------------------------------------------%
            % 1. PARAMETERS CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK THE DIMENSION OF THE VECTOR OF CONTROLS
            if(length(u)~=3)
                str = ['robot.sample_motion_model_odometry:',...
                    '"u" must be of size 3x1'];
                error(str);
            end
            %  1.2. CHECK THE EXISTENCE OF VECTOR OF MOTION ERROR PARAMETERS
            if(isempty(robot.alpha_ODOMETRY))
                str = ['robot.sample_motion_model_odometry:',...
                    '"alpha_ODOMETRY" is not defined'];
                error(str);
            end
            %  1.3. CHECK EXISTENCE OF THE POSE
            if(isempty(robot.x))
                str = ['robot.sample_motion_model_odometry:',...
                    '"x" is not defined'];
                error(str);
            end
            
            %-----------------------------------------------------------%
            % 2. COMPUTE RESULT IN THE CASE OF GAUSSIAN MODEL
            %-----------------------------------------------------------%
            if(strcmp(robot.MOTION_MODEL, 'GAUSSIAN')==1)
                % COMPUTE GAUSSIAN PARAMETERS
                [mu, R] = robot.compute_gaussian_model_odometry(u);
                % TAKE 1 SAMPLE
                N = 1;
                SAMPLES    = mvnrnd(mu,R,N)';
                SAMPLES(3) = pi_to_pi(SAMPLES(3));
                % SET SAMPLE
                xp      = SAMPLES;
                % SAVE NEW POSE
                robot.x = SAMPLES; 
                return
            end
            
            %-----------------------------------------------------------%
            % 3. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  3.1. GET THE POSE OF THE ROBOT AT TIME "t-1"
            xx = robot.x(1);
            yy = robot.x(2);
            theta = robot.x(3);     
            %  3.2. GET THE DELTAS FROM THE ENCODERS
            drot1  = u(1);
            dtrans = u(2);
            drot2  = u(3);
            %  3.3. GET THE ALPHA
            alpha = robot.alpha_ODOMETRY;
            
            %-----------------------------------------------------------%
            % 4. COMPUTE ROTATIONAL MOTION IN THE IMPROVED MODEL
            %-----------------------------------------------------------%     
            if(abs(dtrans)<=robot.DTRANS_ROT &&...
                    strcmp(robot.MOTION_MODEL,'IMPROVED'))
                % SET GAUSSIAN
                mu = [0;0;drot1+drot2];
                R = diag( [alpha(4)*(abs(drot1)+abs(drot2)) ;...
                           alpha(4)*(abs(drot1)+abs(drot2)) ;...
                           alpha(1)*(abs(drot1)+abs(drot2))] );
                % TAKE 1 SAMPLE
                NN = 1;
                samples = mvnrnd(mu,R,NN)';
                xp      = robot.x + samples;
                xp(3)   = pi_to_pi(xp(3));
                % SAVE NEW POSE
                robot.x = xp; 
                return;
            end
            % ROS APPROACH FOR PURE ROTATION
            % https://github.com/ros-planning/navigation2/blob/master/nav2_amcl/src/motion_model/differential_motion_model.cpp
%             if(abs(dtrans)<=robot.DTRANS_ROT)
%                 Dth = drot1+drot2;
%                 drot2 = 1.0*Dth;
%                 drot1 = 0.0*Dth;
%             end
                      
            %-----------------------------------------------------------%
            % 5. COMPUTE THE TRUE "NOISY" DELTAS
            %    We assume that the true values are obtained from the 
            %    measured ones by subtracting independent noise
            %-----------------------------------------------------------%
            %  5.1. COMPUTE THE VARIANCES OF THE NOISE
            switch(robot.MOTION_MODEL)
                case 'STANDARD'
                    var1 = alpha(1)*drot1^2  + alpha(2)*dtrans^2;    
                    var2 = alpha(3)*dtrans^2 + alpha(4)*(drot1^2  + drot2^2);  
                    var3 = alpha(1)*drot2^2  + alpha(2)*dtrans^2;
                case {'STANDARD2', 'IMPROVED'}
                    % Compute absolute values
                    a_drot1  = abs(drot1);
                    a_dtrans = abs(dtrans);
                    a_drot2  = abs(drot2);
                    % Variances
                    var1 = alpha(1)*a_drot1  + alpha(2)*a_dtrans;    
                    var2 = alpha(3)*a_dtrans + alpha(4)*(a_drot1 + a_drot2);  
                    var3 = alpha(1)*a_drot2  + alpha(2)*a_dtrans;
                otherwise
                    str = ['robot.sample_motion_model_odometry:'...
                        'Undefined MOTION_MODEL'];
                    error(str);
            end
            %  5.2. SET THE "true" DELTAS
            drot1_b  = drot1  - sqrt(var1)*randn(1);
            dtrans_b = dtrans - sqrt(var2)*randn(1);
            drot2_b  = drot2  - sqrt(var3)*randn(1);
            
            %-----------------------------------------------------------%
            % 6. COMPUTE THE NEW POSE
            %-----------------------------------------------------------%   
            %  6.1. COMPUTE THE NEW POSE
            xx_p = xx  +  dtrans_b*cos(theta + drot1_b);
            yy_p = yy  +  dtrans_b*sin(theta + drot1_b);
            theta_p = theta  +  drot1_b  +  drot2_b;
            theta_p = pi_to_pi(theta_p);
            %  6.2. RETURN THE NEW POSE
            xp = [xx_p; yy_p; theta_p];
            %  6.3. SET THE NEW POSE OF THE ROBOT
            robot.x = xp;
        end
        
        function [p, drot1_p,dtrans_p,drot2_p, e_rot1,e_trans,e_rot2] =...
                motion_model_odometry(robot, xp, u, m)
            % robot.motion_model_odometry  Transition probability "p(xp|u,x)"
            %
            %   [P, deltas..] = MOTION_MODEL_ODOMETRY(XP, U) Implements the 
            %                   algorithm "motion_model_odometry" that is
            %                   described in the table 5.5. See Page 134
            % 
            % OUTPUTS
            %        p: p(xp | u, x) probability of being at "xp" after 
            %           executing control "u" beginning in state "x", assuming 
            %           that the control is carried for a fixed duration "delta".
            %   drot1_p: The true rotation "drot1" from x to xp
            %  dtrans_p: The true translation "dtrans" from x to xp
            %   drot2_p: The true rotation "drot2" from x to xp
            %    e_rot1: Motion delta error in the first rotation
            %   e_trans: Motion delta error in the translation
            %    e_rot2: Motion delta error in the second rotation
            %
            % INPUTS
            %      xp: Vector that represent the pose "[x;y;theta]" at time "t" 
            %       u: Vector of control [drot1; dtrans; drot2] from the encoders
            %       x: Vector that represent the pose "[x,y,theta]" at time "t-1" 
            %   alpha: Parameters of the motion noise
            
            %-----------------------------------------------------------%
            % 1. PARAMETERS CHECKING
            %-----------------------------------------------------------%
            %  1.1. CHECK THE DIMENSIONS OF THE VECTOR OF CONTROLS
            if(length(u)~=3)
                error('ROBOT.MOTION_MODEL_ODOMETRY: "u" must be of size 3x1') 
            end
            %  1.2. CHECK THE PROPERTY "x"
            if(isempty(robot.x))
                error('ROBOT.MOTION_MODEL_ODOMETRY: "x" is not defined') 
            end
            %  1.3. DEFAULT VALUE OF THE FLAG OF MESSAGES
            if(nargin < 4)
                m=0;
            end      
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1. GET THE DELTAS FROM THE ENCODERS
            drot1  = u(1);
            dtrans = u(2);
            drot2  = u(3);
            %  2.2. GET THE ALPHA
            alpha = robot.alpha_ODOMETRY;
            
            %-----------------------------------------------------------%
            % 3. COMPUTE THE VECTOR OF DELTAS
            %-----------------------------------------------------------%
            u_real = compute_odometry_controls(robot.x, xp, m);
            drot1_p  = u_real(1);
            dtrans_p = u_real(2);
            drot2_p  = u_real(3);
            
            %-----------------------------------------------------------%
            % 4. COMPUTE ROTATIONAL MOTION IN THE IMPROVED MODEL
            %-----------------------------------------------------------%
            if(abs(dtrans)<=robot.DTRANS_ROT &&...
                    strcmp(robot.MOTION_MODEL, 'IMPROVED'))
                % SET GAUSSIAN
                mu    = robot.x + [0;0;drot1+drot2];
                mu(3) = pi_to_pi(mu(3));
                R = diag( [alpha(4)*(abs(drot1)+abs(drot2));...
                           alpha(4)*(abs(drot1)+abs(drot2));...
                           alpha(1)*(abs(drot1)+abs(drot2))] );
                % COMPUTE DENSITIES
                p = gauss_ND_robot(xp,mu,R);
                e_rot1=0; e_trans=0; e_rot2=0;
                return
            end
            
            %-----------------------------------------------------------%
            % 5. COMPUTE RESULT IN THE CASE OF GAUSSIAN MODEL
            %-----------------------------------------------------------%
            if(strcmp(robot.MOTION_MODEL, 'GAUSSIAN')==1)
                % COMPUTE GAUSSIAN PARAMETERS
                [mu, R] = robot.compute_gaussian_model_odometry(u);
                % COMPUTE PROBABILITY
                p = gauss_ND_robot(xp,mu,R);
                e_rot1=0; e_trans=0; e_rot2=0;
                return
            end

            %-----------------------------------------------------------%
            % 6. COMPUTE THE ERRORS IN THE DELTAS
            %-----------------------------------------------------------%
            %  6.1. ERROR IN "drot1"
            e_rot1  = drot1  - drot1_p;
            e_rot1 = pi_to_pi(e_rot1);
            %  6.2. ERROR IN "dtrans"
            e_trans = dtrans - dtrans_p;
            %  6.3. ERROR IN "drot2"
            e_rot2  = drot2  - drot2_p;
            e_rot2 = pi_to_pi(e_rot2);
            
            %-----------------------------------------------------------%
            % 7. COMPUTE THE PROBABILITY "p : p(xp | u, x)"
            %-----------------------------------------------------------%
            %  7.1. COMPUTE THE VARIANCES OF THE NOISE
            switch(robot.MOTION_MODEL)
                case 'STANDARD'
                    var_rot1  = alpha(1)*drot1^2   +  alpha(2)*dtrans^2;    
                    var_trans = alpha(3)*dtrans^2  +  alpha(4)*drot1^2 +...
                                alpha(4)*drot2^2;  
                    var_rot2  = alpha(1)*drot2^2   +  alpha(2)*dtrans^2;
                case {'STANDARD2', 'IMPROVED'}
                    % Compute absolute values
                    a_drot1  = abs(drot1);
                    a_dtrans = abs(dtrans);
                    a_drot2  = abs(drot2);
                    % Variances
                    var_rot1  = alpha(1)*a_drot1  + alpha(2)*a_dtrans ;
                    var_trans = alpha(3)*a_dtrans + alpha(4)*(a_drot1+a_drot2);
                    var_rot2  = alpha(1)*a_drot2  + alpha(2)*a_dtrans;
            end
            %  7.2. COMPUTE THE PROBABILITY OF THE ERRORS (mean = '0')
            p_error_rot1  = gauss_1d(0, var_rot1,  e_rot1);
            p_error_trans = gauss_1d(0, var_trans, e_trans);
            p_error_rot2  = gauss_1d(0, var_rot2,  e_rot2);
            %  7.3. COMPUTE TOTAL PROBABILITY
            p = p_error_rot1*p_error_trans*p_error_rot2;
        end
        
        function [mu, R] = compute_gaussian_model_odometry(this, u)
            % robot.compute_gaussian_model_velocity Gaussian odometry model
            %
            %   COMPUTE_GAUSSIAN_MODEL_ODOMETRY(u,DT) Compute the gaussian
            %   odometry model.
            %
            % OUTPUTS
            %      mu: Mean of the Gaussian motion model
            %       R: Covarianze of the Gaussian motion model
            %
            % INPUTS
            %    u: Vector of controls [drot1;dtrans;drot2]
            
            %-----------------------------------------------------------%
            % 1. GET NOISE VECTOR
            %-----------------------------------------------------------%
            alpha = this.alpha_ODOMETRY;
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1. GET THE CONTROLS
            drot1  = u(1);
            dtrans = u(2);
            drot2  = u(3);
            %  2.2. GET THE ANGLE OF THE CURRENT POSE
            theta = this.x(3);
            %  2.3. GET ABSOLUTE VALUE OF DELTAS
            a_drot1  = abs(drot1);
            a_dtrans = abs(dtrans);
            a_drot2  = abs(drot2);
            
            %----------------------------------------------------------%
            % 3. COMPUTE THE MEAN OF THE MOTION MODEL
            %----------------------------------------------------------%
            mu = noise_free_motion_model_odometry(this.x, u);
            
            %----------------------------------------------------------%
            % 4. COVARIANCE MATRIX
            %----------------------------------------------------------%
            if(abs(dtrans)<=this.DTRANS_ROT) % ROTATIONAL MOTION
                R = diag( [alpha(4)*(abs(drot1)+abs(drot2));...
                           alpha(4)*(abs(drot1)+abs(drot2));...
                           alpha(1)*(abs(drot1)+abs(drot2))] );
            else
                % JACOBIAN WRT SPACE CONTROL
                V = [ -dtrans*sin(theta + drot1)   cos(theta + drot1)   0;
                       dtrans*cos(theta + drot1)   sin(theta + drot1)   0;
                                  1                         0           1];
                % COVARIANCE MATRIX IN CONTROL SPACE
                M = diag([alpha(1)*a_drot1 + alpha(2)*a_dtrans,...
                          alpha(3)*a_dtrans + alpha(4)*(a_drot1 + a_drot2),...
                          alpha(1)*a_drot2 + alpha(2)*a_dtrans]);
                R  = V*M*V';
            end
        end
        %------------- END ODOMETRY MOTION FUNCTIONS ----------------%
        
        
        %--------- PREDICTED BELIEF GAUSSIAN APPROXIMATIONS ---------%
        function set_gaussian_parameters(this, mu, P)
            % robot.set_gaussian_parameters Set the gaussian
            %
            %   SET_GAUSSIAN_PARAMETERS(MU,P) Set the parameters of the
            %   gaussian approximation of the motion model
            
            %-----------------------------------------------------------%
            % 1. SET PARAMETERS
            %-----------------------------------------------------------%
            %  1.1. MAKE SURE "mu" IS A COLUMN
            mu = mu(:);
            %  1.2. CHECK THE ANGLE
            mu(3) = pi_to_pi(mu(3));
            %  1.3. SET PARAMETERS
            this.MU = mu;
            this.PP = P;
        end
        
        function display_gaussian_parameters(this)
            % display_gaussian_parameters
            %
            %   DISPLAY_GAUSSIAN_PARAMETERS Display in the Command Window 
            %   the statistical parameters
            
            %-----------------------------------------------------------%
            % 1. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  1.1. GET PARAMETERS
            mu = this.MU;
            P  = this.PP;
            %  1.2. GET STD's 
            sigmas = sqrt(diag(P));
            
            %-----------------------------------------------------------%
            % 2. DISPLAY MESSAGES
            %-----------------------------------------------------------%
            fprintf(' The gaussian parameters of ''%s'' are:\n',...
                inputname(1))
            fprintf('     mu: [%2.4f, %2.4f, %2.4f(sex°)]\n',...
                mu(1:2), mu(3)*180/pi)
            fprintf('  sigma: [%2.4f, %2.4f, %2.4f(sex°)]\n',...
                sigmas(1:2), sigmas(3)*180/pi)
        end
        
        function [mup,Pp] = prediction_gaussian_model_velocity(this,u,dt)
            % robot.prediction_gaussian_model_velocity
            %
            %   PREDICTION_GAUSSIAN_MODEL_VELOCITY(u,DT) Compute the
            %   predicted belief of the samples under a Gaussian model
            %   velocity
            %
            % OUTPUTS
            %   mup: Mean of the belief
            %    Pp: Covariance matrix
            %
            % INPUTS
            %    u: Vector of controls
            %   dt: Time interval
            %
            % NOTES:
            %   - We update the internal gausian parameters "MU", "PP"
            
            %----------------------------------------------------------%
            % 1. PARAMETERS CHECKING
            %----------------------------------------------------------%
            %  1.1. GET NOISE PARAMETERS
            alpha = this.alpha_VELOCITY;
            %  1.2. GET GAUSSIAN PARAMETERS
            mu = this.MU;
            P  = this.PP;
            
            %----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %----------------------------------------------------------%
            %  2.1. GET THE CONTROLS
            v = u(1);
            w = u(2);
            %  2.2. GET THE ANGLE
            theta = mu(3);
            
            %----------------------------------------------------------%
            % 3. COMPUTE THE PREDICTED MEAN AT TIME "t"
            %     -> We also get in "f" the kind of motion 
            %----------------------------------------------------------%
            [mup, ~, ~,~, f] = noise_free_motion_model_velocity(mu, u, dt);         
            
            %----------------------------------------------------------%
            % 4. COMPUTE THE JACOBIANS NEEDED FOR THE LINEARIZED MODEL
            %----------------------------------------------------------%
            if(f == 0)   % PURE TRANSLATIONAL
                % a. Compute the Jacobian with respect to the state (w=0)
                G = [1   0   -v*(dt*sin(theta)+0);
                     0   1    v*(dt*cos(theta)+0);
                     0   0                   1];
                % b. Compute the Jacobian with respect to the controls
                %   The first column has another term -1/2*w*dt^2*sin(th)
                %   but we can eliminate it
                V = [dt*cos(theta)  -1/2*v*dt^2*sin(theta)    0;
                     dt*sin(theta)   1/2*v*dt^2*cos(theta)    0;
                           0                   dt            dt]; 
            else  % TRANSLATIONAL + ROTACIONAL  
                % a. Compute the Jacobian with respect to the state
                G = [1  0   -(v/w)*cos(theta)+(v/w)*cos(theta + w*dt);
                     0  1   -(v/w)*sin(theta)+(v/w)*sin(theta + w*dt);
                     0  0                                           1];
                % b. Compute the Jacobian with respect to the controls
                c1 = [(-sin(theta) + sin(theta + w*dt))/w;
                      ( cos(theta) - cos(theta + w*dt))/w;
                                    0];
                c2 = [ v*(sin(theta) - sin(theta+w*dt))/w^2  +  v*cos(theta+w*dt)*dt/w;
                      -v*(cos(theta) - cos(theta+w*dt))/w^2  +  v*sin(theta+w*dt)*dt/w;
                                                            dt];
                c3 = [0;0;dt];
                V = [c1  c2  c3];
            end
            
            %----------------------------------------------------------%
            % 5. COMPUTE THE PREDICTED COVARIANCE 'R'
            %----------------------------------------------------------%
            if(abs(u(1))<=1e-4 && (strcmp(this.MOTION_MODEL, 'IMPROVED') ||...
                    strcmp(this.MOTION_MODEL, 'GAUSSIAN')) && 1)
                % IN THE CASE OF PURE ROTATION AND SOME MODELS
                R = diag( [ alpha(2)*(w^2)*dt/1.1412;...
                            alpha(2)*(w^2)*dt/1.4142;...
                           (alpha(4) + alpha(6))*(w^2)*dt] );
            else
                % SET M
                switch(this.MOTION_MODEL)
                    case 'STANDARD'
                        M = diag([alpha(1)*v^2 + alpha(2)*w^2,...
                                  alpha(3)*v^2 + alpha(4)*w^2,...
                                  alpha(5)*v^2 + alpha(6)*w^2]);
                    case {'STANDARD2','IMPROVED','GAUSSIAN'}
                        M = 1/dt*diag([alpha(1)*v^2 + alpha(2)*w^2,...
                                    alpha(3)*v^2 + alpha(4)*w^2,...
                                    alpha(5)*v^2 + alpha(6)*w^2]);
                end
                % COMPUTE MATRIX 'R'
                R = V*M*V';
                % DO THIS FOR THE STANDARD, STANDARD2 MODELS IN PURE
                % ROTATION
                if(abs(u(1))<1e-6)
                    R(1,1) = R(1,1) + 1e-8;
                end
            end
            
            %----------------------------------------------------------%
            % 6. COMPUTE "Pp"
            %----------------------------------------------------------%
            Pp = G*P*G' + R;
            
            %----------------------------------------------------------%
            % 7. UPDATE THE INTERNAL GAUSSIAN
            %----------------------------------------------------------%
            this.MU = mup;
            this.PP = Pp;
        end
        
        function [mup, Pp] = prediction_gaussian_model_odometry(this, u)
            % robot.prediction_gaussian_model_odometry
            %
            %   PREDICTION_GAUSSIAN_MODEL_ODOMETRY(u) Compute the
            %   predicted belief of the samples under a Gaussian model
            %   odometry
            %
            % OUTPUTS
            %   mup: Mean of the Gaussian
            %    Pp: Covarianze
            %
            % INPUTS
            %      u: Vector of odometry controls
            
            %-----------------------------------------------------------%
            % 1. PARAMETERS CHECKING
            %-----------------------------------------------------------%               
            %  1.1. GET NOISE VECTOR
            alpha = this.alpha_ODOMETRY;
            %  1.2. GET GAUSSIAN PARAMETERS
            mu = this.MU;
            P  = this.PP;
            
            %-----------------------------------------------------------%
            % 2. INITIAL CONFIGURATION
            %-----------------------------------------------------------%
            %  2.1. GET THE CONTROLS: "DELTAS" FROM THE ENCODERS
            drot1  = u(1);
            dtrans = u(2);
            drot2  = u(3);
            %  2.2. GET THE ANGLE
            theta = mu(3);
            
            %-----------------------------------------------------------%
            % 3. COMPUTE THE PREDICTED MEAN AT TIME "t"
            %-----------------------------------------------------------%
            mup = noise_free_motion_model_odometry(mu, u);
            
            %-----------------------------------------------------------%
            % 4. COMPUTE THE COVARIANCE MATRIX IN CONTROL SPACE
            %-----------------------------------------------------------%
            if(abs(dtrans)<=this.DTRANS_ROT &&...
                    (strcmp(this.MOTION_MODEL, 'IMPROVED') ||...
                     strcmp(this.MOTION_MODEL, 'GAUSSIAN')) && 1)
                % COVARIANCE MATRIX
                R = diag( [alpha(4)*(abs(drot1)+abs(drot2));...
                           alpha(4)*(abs(drot1)+abs(drot2));...
                           alpha(1)*(abs(drot1)+abs(drot2))] );
            else
                % COMPUTE THE JACOBIAN WITH RESPECT TO THE CONTROLS
                V = [ -dtrans*sin(theta + drot1)   cos(theta + drot1)   0;
                       dtrans*cos(theta + drot1)   sin(theta + drot1)   0;
                                    1                        0          1];
                % COVARIANCE MATRIX
                switch(this.MOTION_MODEL)
                    case 'STANDARD'
                        M = diag([alpha(1)*drot1^2  + alpha(2)*dtrans^2,...
                                  alpha(3)*dtrans^2 + alpha(4)*(drot1^2 + drot2^2),...
                                  alpha(1)*drot2^2  + alpha(2)*dtrans^2 ]);
                    case {'STANDARD2','IMPROVED','GAUSSIAN'}
                        a_drot1  = abs(drot1);
                        a_dtrans = abs(dtrans);
                        a_drot2  = abs(drot2);
                        M = diag([alpha(1)*a_drot1  + alpha(2)*a_dtrans,...
                                  alpha(3)*a_dtrans + alpha(4)*(a_drot1 + a_drot2),...
                                  alpha(1)*a_drot2  + alpha(2)*a_dtrans]);
                end
                R = V*M*V';
            end
            
            %-----------------------------------------------------------%
            % 5. COMPUTE COVARIANCE MATRIX IN STATE SPACE
            %-----------------------------------------------------------%
            %  5.1. COMPUTE THE JACOBIAN WITH RESPECT TO THE STATE
            G = [1  0   -dtrans*sin(theta + drot1);
                 0  1    dtrans*cos(theta + drot1);
                 0  0               1];
            %  5.2. MATRIX
            Pp = G*P*G' + R;
            
            %----------------------------------------------------------%
            % 6. UPDATE THE INTERNAL GAUSSIAN
            %----------------------------------------------------------%
            this.MU = mup;
            this.PP = Pp;
        end
        %------- END PREDICTED BELIEF GAUSSIAN APPROXIMATIONS -------%
        
    end % END PUBLIC METHODS
    
    
    %------------------------------------------------------------------%
    %  PRIVATE METHODS
    %------------------------------------------------------------------%
    methods(Access=private)
        %--------------- R2D2-00, R2D2-01 FUNCTIONS -----------------%
        function use_robot_R2D2(this)
            % robot.use_robot_R2D2 Set parameters of the R2D2 robots
            %
            %   USE_ROBOT_R2D2 Set "main "parameters for the R2D2 family
            %   of robots
            
            %-----------------------------------------------------------%
            % 1. SET MECHANICAL DIMENSIONS OF THE R2D2-00 ROBOT
            %-----------------------------------------------------------%
            %  1.1. SET PARAMETERS
            switch(this.NAME_ROBOT)
                case 'R2D2-00'
                    Length   =  40/100;        % Length of the vehicle [m]
                    Width    =  30/100;        % Width of the vehicle [m] 
                    Eje      =   6/100;        % Distance from the axis to the front
                    Diameter = 9.8/100;        % Diameter of the wheels [m]
                    WW       = (2.5+1.3)/100;  % "total" width  of the wheels
                case 'R2D2-01'
                    Length   =  30/100;
                    Width    =  25/100;
                    Eje      =   6/100;
                    Diameter = 9.8/100;
                    WW       = (2.5+1.3)/100;
            end
            %  1.2. SAVE STRUCTURE
            this.DIMENSIONS.Length   = Length;
            this.DIMENSIONS.Width    = Width;
            this.DIMENSIONS.Eje      = Eje;
            this.DIMENSIONS.Diameter = Diameter;
            this.DIMENSIONS.WW       = WW;
            
            %-----------------------------------------------------------%
            % 2. SET EQUIVALENT "main" DIMENSIONS
            %-----------------------------------------------------------%
            %  2.1. SET DISTANCE FROM THE CENTER TO THE WHEELS
            BB = Width + WW;
            this.B = 0.5*BB;
            %  2.2. SET DIAMETERS AND RADIOUS OF THE WHEELS
            this.D = Diameter;
            this.r = this.D/2;
        end
        
        function display_R2D2(this, handle_car)
            % robot.display_R2D2 Display the R2D2-00 robot
            %
            %   DISPLAY_R2D2(HANDLE_CAR) Display the R2D2-00 
            %   robot using the handles
            %
            % INPUTS
            %   handle_car: Handlers to fill 2D polygons and draw lines
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS CHECKING
            %-----------------------------------------------------------%           
            %  1.1. CHECK THE HANDLES
            if(length(handle_car) ~= 6)
                str = ['robot.display_R2D2:',...
                    'The handle matrix must be of size 6'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. GET MECHANICAL PARAMETERS 
            %-----------------------------------------------------------%
            LENGTH = this.DIMENSIONS.Length;   % Length of the vehicle [m]
            WIDTH  = this.DIMENSIONS.Width;    % Width of the vehicle [m] 
            EJE    = this.DIMENSIONS.Eje;      % Distance from the axis to the front
            DD     = this.DIMENSIONS.Diameter; % Diameter of the wheels[m]
            ww     = this.DIMENSIONS.WW;       % "Total Width" of the wheels
            
            %-----------------------------------------------------------%
            % 3. GET THE POSITION AND ORIENTATION OF THE ROBOT
            %-----------------------------------------------------------%
            position    = this.x(1:2)';       % Position 
            orientation = this.x(3);          % Orientation
            
            %-----------------------------------------------------------%
            % 4. COMPUTE THE COORDINATES OF EACH COMPONENT OF THE ROBOT
            %-----------------------------------------------------------%
            %  4.1 PRINCIPAL BODY
            %    -> Set the coordinates of the corners in the local coord.
            x_car = [-(LENGTH-EJE)   EJE   EJE  -(LENGTH-EJE)];
            y_car = WIDTH*[ 0.5   0.5   -0.5   -0.5];
            %   -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x_car,y_car);
            %   -> Convert these back to cartesian coord. but with the new 
            %      orientation
            [x_car,y_car] = pol2cart(ang + orientation, radio);
            %   -> Set the new positions of the corners
            x_car = x_car + position(1);
            y_car = y_car + position(2);
            
            %  4.2. WHEEL NUMBER 1 - RIGHT ONE "motor 1"
            %    -> Set the coordinates of the corners relative to the 
            %       local coordinate frame.
            x1_wheels = [-DD/2   DD/2   DD/2   -DD/2];
            y1_wheels = [-0.5*WIDTH   -0.5*WIDTH...
                -(0.5*WIDTH+ww)  -(0.5*WIDTH+ww)];
            %    -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x1_wheels,y1_wheels);
            %    -> Convert these back to cartesian coor but with the new
            %       orientation
            [x1_wheels,y1_wheels] = pol2cart(ang + orientation, radio);
            %    -> Set the new positions of the corners
            x1_wheels = x1_wheels + position(1);
            y1_wheels = y1_wheels + position(2);
            
            %  4.3. WHEEL NUMBER 2  - LEFT ONE "motor 2"
            %    -> Set the coordinates of the corners relative to the 
            %       local coordinate frame.
            x2_wheels = [-DD/2   DD/2   DD/2   -DD/2];
            y2_wheels = [0.5*WIDTH+ww   0.5*WIDTH+ww ...
                            0.5*WIDTH     0.5*WIDTH];
            %   -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x2_wheels,y2_wheels);
            %   -> Convert these back to cartesian coor but with the new 
            %      orientation
            [x2_wheels,y2_wheels] = pol2cart(ang + orientation, radio);
            %    -> Set the new positions of the corners
            x2_wheels = x2_wheels + position(1);
            y2_wheels = y2_wheels + position(2);
            
            %  4.4. POSTERIOR WHEEL - Rueda loca
            %   -> Set parameters of the wheel
            dd = LENGTH - EJE - 0.05;   % Distance from the local frame
            rr = 0.02;                  % Radious of the wheel        
            %    -> Set representative points of the circle(robot frame)
            NN = 24;  
            ang = linspace(0,2*pi,NN)';
            x3_wheels = -dd*ones(NN,1) + rr*cos(ang);    % x-components
            y3_wheels =  0.0*ones(NN,1) + rr*sin(ang);   % y_components
            %   -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x3_wheels,y3_wheels);
            %   -> Convert these back to cartesian coor but with the new 
            %      orientation
            [x3_wheels,y3_wheels] = pol2cart(ang + orientation, radio);
            %    -> Set the circle in the global frame
            x3_wheels = x3_wheels + position(1);
            y3_wheels = y3_wheels + position(2);
            
            %-----------------------------------------------------------%
            % 5. SET THE AXIS OF THE ROBOT
            %-----------------------------------------------------------%
            %  5.1 LINE 1 Represent the x-axis of the local frame
            %    -> Set the coordinates of the line
            x_line1 = [-3/100  10/100];
            y_line1 = [   0      0 ];
            %   -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x_line1,y_line1);
            %   -> Convert these back to cartesian coord. but with the new 
            %      orientation
            [x_line1,y_line1] = pol2cart(ang + orientation, radio);
            %   -> Set the new location
            x_line1 = x_line1 + position(1);
            y_line1 = y_line1 + position(2);
                
            %  5.2 LINE 2 Represent the y-axis of the local frame
            %    -> Set the coordinates of the line
            x_line2 = [   0     0];
            y_line2 = [-3/100  10/100 ];
            %   -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x_line2,y_line2);
            %   -> Convert these back to cartesian coord. but with the
            %      new orientation
            [x_line2,y_line2] = pol2cart(ang + orientation, radio);
            %   -> Set the new location
            x_line2 = x_line2 + position(1);
            y_line2 = y_line2 + position(2); 
            
            %-----------------------------------------------------------%
            % 6. DRAW THE ROBOT
            %-----------------------------------------------------------%
            %  6.1. DRAW THE POLYGONS
            set(handle_car(1),'xdata',x_car,'ydata',y_car,'facecolor','y');
            set(handle_car(2),'xdata',x1_wheels,'ydata',y1_wheels,'facecolor','g');
            set(handle_car(3),'xdata',x2_wheels,'ydata',y2_wheels,'facecolor','g');
            set(handle_car(4),'xdata',x3_wheels,'ydata',y3_wheels,'facecolor','r');
                        %  6.2. DRAW THE LINES
            set(handle_car(5),'xdata', x_line1,'ydata',y_line1,'Color','k');
            set(handle_car(6),'xdata', x_line2,'ydata',y_line2,'Color','k');          
        end 
        %------------------------------------------------------------%
        
        %-------------------- R2D2-RX FUNCTIONS ---------------------%
        function use_robot_R2D2_RX(this)
            % robot.use_robot_R2D2_RX Set parameters of the R2D2-RX robots
            %
            %   USE_ROBOT_R2D2_RX Set "main "parameters for the R2D2-RX
            %   family of robots
            
            %-----------------------------------------------------------%
            % 1. SET MECHANICAL DIMENSIONS OF THE R2D2-RX ROBOT
            %-----------------------------------------------------------%
            switch(this.NAME_ROBOT)
                case 'R2D2-R1'
                    % BODY 1
                    this.DIMENSIONS.L1 = 0.30;
                    this.DIMENSIONS.L2 = 0.12;
                    % BODY 2
                    this.DIMENSIONS.L3 = 0.16;
                    this.DIMENSIONS.L4 = 0.12;      % 23.8cm - 12cm
                    % DISTANCE FROM THE CENTER TO THE WHEELS
                    this.B = 0.235/2;
                    % DIAMETERS AND RADIOUS OF THE WHEELS
                    this.D = 0.098;
                    this.r = this.D/2;
                    
                case 'R2D2-R2'
                    % BODY 1
                    this.DIMENSIONS.L1 = 0.45;
                    this.DIMENSIONS.L2 = 0.20;
                    % BODY 2
                    this.DIMENSIONS.L3 = 0.24;
                    this.DIMENSIONS.L4 = 0.18;      % 38cm - 20cm
                    % DISTANCE FROM THE CENTER TO THE WHEELS
                    this.B = 0.34/2;
                    % DIAMETERS AND RADIOUS OF THE WHEELS
                    this.D = 0.122;
                    this.r = this.D/2;
            end
        end
        
        function display_R2D2_RX(this, handle_car)
            % robot.display_R2D2_RX Display the R2D2-RX robot
            %
            %   DISPLAY_R2D2_RX(HANDLE_CAR) Display the R2D2-RX robot 
            %   using the handles
            %
            % INPUTS
            %   handle_car: Handles to fill 2D polygons and draw lines
            
            %-----------------------------------------------------------%
            % 1. ARGUMENTS CHECKING
            %-----------------------------------------------------------%           
            %  1.1. CHECK THE HANDLES
            if(length(handle_car) ~= 6)
                str = ['robot.display_R2D2_RX:',...
                    'The handle matrix must be of size 6'];
                error(str)
            end
            
            %-----------------------------------------------------------%
            % 2. GET MECHANICAL PARAMETERS 
            %-----------------------------------------------------------%
            %  2.1. LENGTH OF THE BODIES
            L1 = this.DIMENSIONS.L1;        % 
            L2 = this.DIMENSIONS.L2;        % 0.20
            L3 = this.DIMENSIONS.L3;        % 
            L4 = this.DIMENSIONS.L4;        % 0.18
            %  2.2. MAIN REPRESENTATIVE DIMENSIONS
            BB  = this.B;
            DD  = this.D;
            %  2.3. WIDTH OF THE WHEELS
            ww = 0.024;
                        
            %-----------------------------------------------------------%
            % 3. GET THE POSITION AND ORIENTATION OF THE ROBOT
            %-----------------------------------------------------------%
            position    = this.x(1:2)';       % Position 
            orientation = this.x(3);          % Orientation
            
            %-----------------------------------------------------------%
            % 4. COMPUTE THE COORDINATES OF EACH COMPONENT OF THE ROBOT
            %-----------------------------------------------------------%
            %  4.1 SET BODY1
            %    -> Set the coordinates of the corners in the local frame
            x_body1 = [-L2/2   L2/2    L2/2  -L2/2];
            y_body1 = [ L1/2   L1/2   -L1/2  -L1/2];
            %   -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x_body1,y_body1);
            %   -> Convert these back to cartesian coord. but with the 
            %      new orientation
            [x_body1,y_body1] = pol2cart(ang + orientation, radio);
            %   -> Set the new positions of the corners
            x_body1 = x_body1 + position(1);
            y_body1 = y_body1+ position(2);
            
            %  4.2 SET BODY2
            %    -> Set the coordinates of the corners in the local frame
            x_body2 = [-L4-L2/2   -L2/2   -L2/2  -L4-L2/2];
            y_body2 = [  L3/2     L3/2    -L3/2   -L3/2];
            %   -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x_body2,y_body2);
            %   -> Convert these back to cartesian coord. but with the
            %      new orientation
            [x_body2,y_body2] = pol2cart(ang + orientation, radio);
            %   -> Set the new positions of the corners
            x_body2 = x_body2 + position(1);
            y_body2 = y_body2 + position(2);
            
            %  4.3. WHEEL NUMBER 1 - RIGHT ONE "motor 1"
            %    -> Set the coordinates of the corners relative to the 
            %       local coordinate frame.
            x1_wheels = [   -DD/2        DD/2       DD/2      -DD/2];
            y1_wheels = [-(BB-ww/2)  -(BB-ww/2)  -(BB+ww/2)  -(BB+ww/2)];
            %    -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x1_wheels,y1_wheels);
            %    -> Convert these back to cartesian coor but with the
            %       new orientation
            [x1_wheels,y1_wheels] = pol2cart(ang + orientation, radio);
            %    -> Set the new positions of the corners
            x1_wheels = x1_wheels + position(1);
            y1_wheels = y1_wheels + position(2);
            
            %  4.2. WHEEL NUMBER 2 - LEFT ONE "motor 2"
            %    -> Set the coordinates of the corners relative to the 
            %       local coordinate frame.
            x2_wheels = [  -DD/2     DD/2     DD/2     -DD/2];
            y2_wheels = [(BB+ww/2)  (BB+ww/2)  (BB-ww/2)  (BB-ww/2)];
            %    -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x2_wheels,y2_wheels);
            %    -> Convert these back to cartesian coor but with the
            %       new orientation
            [x2_wheels,y2_wheels] = pol2cart(ang + orientation, radio);
            %    -> Set the new positions of the corners
            x2_wheels = x2_wheels + position(1);
            y2_wheels = y2_wheels + position(2);
            
            %-----------------------------------------------------------%
            % 5. SET THE AXIS OF THE ROBOT
            %-----------------------------------------------------------%
            %  5.1 LINE 1 Represent the x-axis of the local frame
            %    -> Set the coordinates of the line
            x_line1 = [-3/100  10/100];
            y_line1 = [   0      0 ];
            %   -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x_line1,y_line1);
            %   -> Convert these back to cartesian coord. but with the
            %      new orientation
            [x_line1,y_line1] = pol2cart(ang + orientation, radio);
            %   -> Set the new location
            x_line1 = x_line1 + position(1);
            y_line1 = y_line1 + position(2);
                
            %  5.2 LINE 2 Represent the y-axis of the local frame
            %    -> Set the coordinates of the line
            x_line2 = [   0     0];
            y_line2 = [-3/100  10/100 ];
            %   -> Convert these to polar coordinates
            [ang, radio] = cart2pol(x_line2,y_line2);
            %   -> Convert these back to cartesian coord. but with the 
            %      new orientation
            [x_line2,y_line2] = pol2cart(ang + orientation, radio);
            %   -> Set the new location
            x_line2 = x_line2 + position(1);
            y_line2 = y_line2 + position(2);
            
            %-----------------------------------------------------------%
            % 6. DRAW THE ROBOT
            %-----------------------------------------------------------%
            %  6.1. DRAW THE POLYGONS
            set(handle_car(1),'xdata',x_body1,'ydata',y_body1,'facecolor','y');
            set(handle_car(2),'xdata',x_body2,'ydata',y_body2,'facecolor','y');
            set(handle_car(3),'xdata',x1_wheels,'ydata',y1_wheels,'facecolor','g');
            set(handle_car(4),'xdata',x2_wheels,'ydata',y2_wheels,'facecolor','g');
            %  6.2. DRAW THE LINES
            set(handle_car(5),'xdata', x_line1,'ydata',y_line1,'Color','k');
            set(handle_car(6),'xdata', x_line2,'ydata',y_line2,'Color','k'); 
        end
        %------------------------------------------------------------%
    end % END PRIVATE METHODS
    
    
    %------------------------------------------------------------------%
    %  STATIC METHODS
    %------------------------------------------------------------------%
    methods(Static)
        function credits()
            % robot.credits Show credits
            %
            %   CREDITS Show credits
            
            %----------------------------------------------------------%
            % 1. MESSAGES
            %----------------------------------------------------------%
            disp('-------------------------------------')
            disp('-  ABOUT THE "R2D2" MOTION LIBRARY  -')
            disp('-------------------------------------')
            disp('  Author: Ivan A. Calle Flores')
            disp('  e-mail: ivan.calle.flores@gmail.com')
        end
    end
end

