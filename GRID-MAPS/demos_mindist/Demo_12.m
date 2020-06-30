% Demo_12: Compute minimum distances using a new local grid map
% Author : Ivan A. Calle Flores
%  NOTE: The conclusion is to pass as large region as posible that 
%        encompass the obstacle to get better precision
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
tic
my_grid.compute_matrix_min_distance();
toc
%  3.3 PLOT THE MATRIX OF MINIMUM DISTANCES
my_grid.plot_matrix_min_distance();


%----------------------------------------------------------------------%
% 4. COMPUTE MINIMUM DISTANCES IN A GIVEN GRID_LOCAP (PLANNING) 
%----------------------------------------------------------------------%
%  4.1. SET LOCAL GRID MAP
SIZE = 20;
GRID_LOCAL = zeros(SIZE);
%  Columns
GRID_LOCAL(3:end-2, 7:8) = 1;
GRID_LOCAL(3:end-2, end-8:end-5) = 1;
%  Rows
GRID_LOCAL(4:5, 3:end-2) = 1;
GRID_LOCAL(end-7:end-6, 3:end-2) = 1;
%  4.2. SET INDEXES OF LOCAL MAP IN THE GRID MAP
GRID_IND.xmin_idx = 50;
GRID_IND.ymin_idy = 80;
GRID_IND.xmax_idx = GRID_IND.xmin_idx + SIZE - 1;
GRID_IND.ymax_idy = GRID_IND.ymin_idy + SIZE - 1;
%  4.3. COMPUTE LOCAL DISTANCES
for i=1:5
    tic
    DIST_LOCAL = my_grid.compute_matrix_min_distance(GRID_LOCAL, GRID_IND);
    toc
end
%  4.4. COMPUTE FULL MATRIX OF DISTANCES
DIST_FULL = my_grid.grid_map_min_distance;
DIST_FULL(GRID_IND.ymin_idy:GRID_IND.ymax_idy, GRID_IND.xmin_idx:GRID_IND.xmax_idx) = ...
    DIST_LOCAL;
%  4.5. PLOT RESULTS
figure
imagesc(DIST_FULL, 'XData',my_grid.grid_xx, 'YData', my_grid.grid_yy);
set(gca, 'Ydir', 'normal');
colorbar

% GRID DISTANCES LOCAL
figure
% Local map
ax1 = subplot(1,2,1);
imagesc(1-GRID_LOCAL, 'XData', my_grid.grid_xx(GRID_IND.xmin_idx:GRID_IND.xmax_idx),...
    'YData', my_grid.grid_yy(GRID_IND.ymin_idy:GRID_IND.ymax_idy));
set(gca, 'Ydir', 'normal');
colormap(ax1, gray)
xlabel('(a)')
% Minimum distances
ax2 = subplot(1,2,2);
imagesc(DIST_LOCAL, 'XData', my_grid.grid_xx(GRID_IND.xmin_idx:GRID_IND.xmax_idx),...
    'YData', my_grid.grid_yy(GRID_IND.ymin_idy:GRID_IND.ymax_idy));
set(gca, 'Ydir', 'normal');
colorbar
xlabel('(b)')
% print -depsc figure.eps