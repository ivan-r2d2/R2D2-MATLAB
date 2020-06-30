% Demo_03: Compute robot body coordinates in the grid 
%  Author: Ivan A. Calle Flores
clc; clearvars; close all

%----------------------------------------------------------------------%
% MAIN VARIABLES
%----------------------------------------------------------------------%
grid_file = 'map_01e.yaml';
NAME_ROBOT   = 'R2D2-R1';   % 'R2D2-00', 'R2D2-01', 'R2D2-R1', 'R2D2-R2'
MOTION_MODEL = 'STANDARD';  % 'STANDARD', 'IMPROVED', 'GAUSSIAN'
LEVEL_NOISE  = 3;
path = '/home/ivan/IVAN/R2D2/TOOLBOX/MAPS/';


%----------------------------------------------------------------------%
% 1. ENVIRONMENT INITIALIZATION
%----------------------------------------------------------------------%
%  1.1. CREATE A GRID
my_grid = occ_grid([path grid_file]);
my_grid.set_robot(NAME_ROBOT);
%  1.2. CREATE A ROBOT
my_robot = robot();
my_robot.set_robot(NAME_ROBOT)
my_robot.set_motion_model(MOTION_MODEL)
my_robot.set_level_of_noise(LEVEL_NOISE)


%----------------------------------------------------------------------%
% 2. DRAW THE GRID MAP
%----------------------------------------------------------------------%
%  2.1. CREATE THE GRID FIGURE
world.visibility = 'ON';
world.title      = 'Occupancy grid';
world.position   = [10 50 600 500];
my_grid.create_figure(world)
%  2.2. DRAW THE GRID MAP AND THE MAP
my_grid.plot_occupancy_grid()


%----------------------------------------------------------------------%
% 3. INITIAL CONFIGURATION OF THE ROBOT
%----------------------------------------------------------------------%
%  3.1. INITIAL POSE OF THE ROBOT
xx = 4.0;
yy = 5.0;
theta = -0*7*pi/8;
x = [xx; yy; theta];
my_robot.set_pose(x);
%  3.2. DISPLAY ROBOT
%my_grid.display_robot(x)
my_robot.display_vehicle(my_grid.h_vehicle);
%  3.3. DISPLAY ROBOT COORDINATES
my_grid.get_robot_index_coordinates(x);
my_grid.plot_occupancy_grid()
%  3.4. DELTA OF TIME
dt = 0.5;



%----------------------------------------------------------------------%
% 4. MOTION SIMULATION
%----------------------------------------------------------------------%
N = 10;
for i = 1:N
    % A. SET CONTROLS
    v = 0.25;           % Translational velocity (m/s)
    w = pi/4;              % Rotational velocity (rad/s)
    u = [v; w];
    % B. COMPUTE "real" MOTION
    xp = my_robot.sample_motion_model_velocity(u, dt);
    my_robot.display_vehicle(my_grid.h_vehicle);
    %my_grid.display_robot(xp)
    % C. COMPUTE ROBOT BODY COORDINATES IN THE GRID MAP
    my_grid.get_robot_index_coordinates(xp);
    my_grid.plot_occupancy_grid()
    % D. PAUSE
    %disp('Press any key')
    pause(0.2)
end


%----------------------------------------------------------------------%
% 5. DISPLAY RESULTS
%----------------------------------------------------------------------%
disp(' ')
my_robot.display_pose()
%my_robot.plot_robot('-r')