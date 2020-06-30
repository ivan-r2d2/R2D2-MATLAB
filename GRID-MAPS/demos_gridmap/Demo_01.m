% Demo_01: Plot a grid map and display a robot
%  Author: Ivan A. Calle Flores
clc; clearvars; close all

%----------------------------------------------------------------------%
% MAIN VARIABLES
%----------------------------------------------------------------------%
grid_file = 'map_01e.yaml';
path = '/home/ivan/IVAN/R2D2/TOOLBOX/MAPS/';


%----------------------------------------------------------------------%
% 1. ENVIRONMENT INITIALIZATION
%----------------------------------------------------------------------%
%  1.1. CREATE AND OBJECT "occ_grid " WITH ITS ASSOCIATED GRID MAP
my_grid = occ_grid([path grid_file]);
my_grid.display_grid_properties()


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
% 3. SET A ROBOT
%----------------------------------------------------------------------%
%  3.1. INITIAL POSE OF THE ROBOT
xx = 4.0;
yy = 5.0;
theta = -0*7*pi/8;
x = [xx; yy; theta];
%  3.2. DISPLAY ROBOT
%my_grid.display_robot(x)

% print -depsc figure.eps