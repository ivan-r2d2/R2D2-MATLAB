% Demo_10: Compute and save the matrix of minimum distances
% Author : Ivan A. Calle Flores
clc; clearvars; close all

%----------------------------------------------------------------------%
% MAIN VARIABLES
%----------------------------------------------------------------------%
name_map  = 'map_01';
grid_file = 'map_01e.yaml';
distances_file = [grid_file(1:end-5) '_min_distances.mat'];
path = '/home/ivan/IVAN/R2D2/TOOLBOX/MAPS/';


%----------------------------------------------------------------------%
% 1. ENVIRONMENT INITIALIZATION
%----------------------------------------------------------------------%
%  1.1. CREATE AN OBJECT "my_laser" WITH ITS ASSOCIATED MAP
my_laser = laser([path name_map]);
%  1.2. CREATE AN OBJECT "my_grid" WITH ITS ASSOCIATED GRID MAP
my_grid = occ_grid(grid_file);


%----------------------------------------------------------------------%
% 2. DRAW THE MAP AND GRID MAP
%----------------------------------------------------------------------%
my_grid.plot_occupancy_grid()
my_laser.plot_map()


%----------------------------------------------------------------------%
% 3. COMPUTE THE MINIMUM DISTANCE FROM A POINT TO A MAP
%----------------------------------------------------------------------%
%  3.1 SET MAXIMUM DISTANCE
MAXIMUM_DISTANCE = 0.4;
my_grid.set_maximum_distance(MAXIMUM_DISTANCE);
%  3.2. COMPUTE MATRIX OF "SQUARED" MIN DISTANCES
N=20;
TT = 0;
for i=1:N
    tic
    my_grid.compute_matrix_min_distance();
    t=toc;
    TT = TT + t;
end
fprintf(' Time to compute matrix of minimum distances[s]: %2.4f. \n',TT/N)

%----------------------------------------------------------------------%
% 4. PLOT THE MATRIX OF MINIMUM DISTANCES
%----------------------------------------------------------------------%
my_grid.plot_matrix_min_distance();


%----------------------------------------------------------------------%
% 5. SAVE THE MATRIX OF MINIMUM DISTANCES - CHANGE NAMES HERE
%----------------------------------------------------------------------%
savefile = [path distances_file];
%my_grid.save_matrix_min_distance(savefile)