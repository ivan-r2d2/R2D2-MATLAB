% Demo_02: Simulation of laser measurement - Interactive
%  Author: Ivan A. Calle Flores
clc; clearvars; close all

%----------------------------------------------------------------------%
% MAIN VARIABLES
%----------------------------------------------------------------------%
name_map  = 'map_01.mat';
path = '/home/ivan/IVAN/R2D2/TOOLBOX/MAPS/';
NAME_ROBOT = 'R2D2-00';
NOISE = 1;


%----------------------------------------------------------------------%
% 1. ENVIROMENT INITIALIZATION
%----------------------------------------------------------------------%
%  1.1. CREATE AN OBJECT "my_laser" WITH ITS ASOCIATED MAP
my_laser = laser([path, name_map]);
%  1.2. CREATE AN OBJECT "my_robot"
my_robot = robot();
my_robot.set_robot(NAME_ROBOT);


%----------------------------------------------------------------------%
% 2. FIGURE CONFIGURATION
%----------------------------------------------------------------------%
%  2.1. CONFIGURE THE MAIN FIGURE
fig1 = figure('Visible','off');    
set(fig1,'position',[10 50 400 400],'Name','MAP: Laser simulation',...
    'color',[211 208 200]/255,'Visible','on');
%   -> Additional properties
set(gca, 'Box', 'on')
hold on
xlabel('x(m)')
ylabel('y(m)')
%  2.2. SET HANDLES TO PLOT THE ROBOT
handle_car = zeros(1,6);
for f=1:4
    handle_car(1,f) = fill(1, 1,'w');
end
handle_car(1,5) = line(0,0);
handle_car(1,6) = line(0,0);


%----------------------------------------------------------------------%
% 3. DRAW THE MAP
%----------------------------------------------------------------------%
my_laser.plot_map()



%----------------------------------------------------------------------%
% 4. LASER RANGE FINDER SPECIFICATIONS
%   -> We simulate a range finder sensor with angles [-pi/2, pi/2]
%----------------------------------------------------------------------%
%  4.1. CONFIGURATIONS OF THE LASER BEAMS
K = 60;         % Number of beams
my_laser.set_mode_operation('NORMAL',K)
%  4.2. SET PARAMETERS
my_laser.set_max_range(15.6);           % Maximum range 
my_laser.set_std_local_noise(0.04);     % Std of. local measurement noise
my_laser.set_short_noise(0.05);         % Probability of unexpected objects
my_laser.set_max_noise(0.05);           % Probability of failures (max.)
%  4.3. UBICATION WRT THE ROBOT'S FRAME (Pg. 169)
xk = 0.1;              % Axis in the direction of motion
yk = 0.1;              % Axis perpendicular to the motion
thk = 1*pi/2;          % Orientation with respect to axis "xk"
my_laser.set_local_pose([xk, yk, thk]);
%  4.4. SET HANDLES TO DRAW THE LASER BEAMS
handle_laser = zeros(1,K+1);
for f=1:K
    handle_laser(1,f) = line(0,0,'Color','m');
end
handle_laser(K+1) = plot(0,0,'+k', 'MarkerSize', 5, 'LineWidth',2);
%  4.5. CREATE A FIGURE TO PLOT THE "END-POINTS"
my_laser.create_laser_figure();


%----------------------------------------------------------------------%
% 5. INITIAL ROBOT CONFIGURATION
%----------------------------------------------------------------------%
%  5.1 SET THE INITIAL POSE
xx = 8; 
yy = 7;
theta = 2*pi/8;
x = [xx; yy; theta];
my_robot.set_pose(x);
%  5.2 DISPLAY THE ROBOT USING THE SPECIFIED HANDLERS
my_robot.display_vehicle(handle_car);
%  5.3 UPDATE TIME
dt = 0.5;
%  5.4 INITIAL LASER MEASUREMENT
[s_full, s] = my_laser.sense_using_laser(x, NOISE);
my_laser.display_laser(x, s_full, handle_laser)
my_laser.display_laser_end_points(s_full)


%----------------------------------------------------------------------%
% 6. COMPUTE THE MOTION
%----------------------------------------------------------------------%
N = 20;          % Set number of iterations
for i = 1:N
    %-------------------------------------------------------------%
    % A. SET THE CONTROLS
    %-------------------------------------------------------------%
    clc
    fprintf('Set the controls for step t = %d\n', i);
    % Translational
    v = 0.2;
    % Rotational
    w = input('  Rotational velocity [pi/8(rad/s)]: ');
    if isempty(w)
        w = pi/8;
    end
    % Vector of controls
    u = [v; w];
    
    
    %-------------------------------------------------------------%
    % B. COMPUTE THE MOTION
    %-------------------------------------------------------------%
    [xp,xc,yc,r,f] = my_robot.noise_free_motion_model_velocity(u, dt);
    my_robot.display_vehicle(handle_car);
    
    
    %-------------------------------------------------------------%
    % C. DO THE MEASUREMENTS FROM "xp" USING THE LASER
    %-------------------------------------------------------------%
    %  Get laser
    [s_full, s] = my_laser.sense_using_laser(xp, NOISE);
    %  Plot results
    my_laser.display_laser(xp, s_full, handle_laser)
    my_laser.display_laser_end_points(s_full)
end