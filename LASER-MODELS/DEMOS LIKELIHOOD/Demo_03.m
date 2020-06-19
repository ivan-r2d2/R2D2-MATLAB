% Demo_03: Plot the likelihood function p(z|x) in 2D
%  Author: Ivan A. Calle Flores
%
% NOTE:
%   - The big lesson here is that the function p(z|x) for a laser
%     range-finder is non-smooth
%   - The maximum "p(z|x)" does not coincide with the true "x"
%   - It is better to use "p(z|x)^ALPHA"
clc; clearvars; close all

%----------------------------------------------------------------------%
% MAIN VARIABLES
%----------------------------------------------------------------------%
name_map  = 'map_01';
grid_file = 'map_01e.yaml';
path = '/home/ivan/IVAN/R2D2/TOOLBOX/MAPS/';
distances_file = [path grid_file(1:end-5) '_min_distances.mat'];
MAPPER = 0;       % "0:LOCALIZATION"


%----------------------------------------------------------------------%
% 1. ENVIRONMENT INITIALIZATION
%----------------------------------------------------------------------%
%  1.1. CREATE AN OBJECT "my_robot" OF THE TYPE R2D2-01
my_robot = robot;
my_robot.set_robot('R2D2-00');
%  1.2. CREATE AN OBJECT "my_laser" WITH ITS ASSOCIATED MAP
my_laser = laser([path name_map]);
%  1.3. CREATE AN OBJECT "my_grid" WITH THE GRID MAP
my_grid = occ_grid(grid_file);
my_grid.set_robot('R2D2-00');
my_grid.set_grid_map_min_distance(distances_file);
%  1.4. CREATE AN OBJECT "my_rfinder" WITH ITS ASSOCIATED GRID MAP
my_rfinder = rfinder();
my_rfinder.set_occ_grid(my_grid)


%----------------------------------------------------------------------%
% 2. PLOT THE OCCUPANCY GRID MAP
%----------------------------------------------------------------------%
my_grid.plot_occupancy_grid()
my_laser.plot_map('r')


%----------------------------------------------------------------------%
% 3. LASER RANGE FINDER SPECIFICATIONS
%----------------------------------------------------------------------%
%  3.1. CONFIGURATIONS OF THE LASER BEAMS
my_laser.set_mode_operation('NORMAL', 500)
%  3.2. SET SELECTED BEAMS FOR LIKELIHOOD MODEL
KK = 40;
THETA_LASER_SELECTED = my_laser.set_theta_selected_index(KK);
my_grid.set_laser_handles(KK);
%  3.3. SET OTHER PROPERTIES OF THE OBJECT "my_laser"
my_laser.set_max_range(5.6);            % Maximum range 
my_laser.set_std_local_noise(0.04);     % Std of. local measurement noise
my_laser.set_short_noise(0.05);         % Probability of unexpected objects
my_laser.set_max_noise(0.05);           % Probability of failures (max.)
%  3.4. UBICATION WRT THE ROBOT'S FRAME (Pg. 169)
xk = 0.04; 
yk = 0.0;
thk = 0*pi/2;
my_laser.set_local_pose([xk,yk,thk]);


%----------------------------------------------------------------------%
% 4. LIKELIHOOOD FIELD SPECIFICATIONS
%----------------------------------------------------------------------%
%  4.1. LIKELIHOOOD FIELD PARAMETERS
if(MAPPER==1)  % According to ROS
    zhit  = 0.90;
    zrand = 0.05;
    zmax  = 0.05; 
    sigma_hit = 0.08;
else
    % For localization
    zhit  = 0.85;
    zrand = 0.05;
    zmax  = 0.05; 
    sigma_hit = 0.12;
end
my_rfinder.set_likelihood_field_parameters(zhit, zrand, zmax, sigma_hit);
%  4.2. LASER PARAMETERS
my_rfinder.set_theta_laser_LIKELIHOOD(THETA_LASER_SELECTED);
my_rfinder.set_laser_local_pose([xk, yk, thk]);
my_rfinder.set_max_range(my_laser.max_range);


%----------------------------------------------------------------------%
% 5. INITIAL ROBOT CONFIGURATION
%----------------------------------------------------------------------%
%  5.1 SET THE INITIAL POSE
disp(' Set the initial position')
[xx, yy] = ginput(1);
%xx = 8; 
%yy = 7;
theta = 4*pi/8;
x = [xx; yy; theta];
my_robot.set_pose(x);
%  5.2. DISPLAY THE ROBOT USING THE SPECIFIED HANDLERS
my_robot.display_vehicle(my_grid.h_vehicle);


%----------------------------------------------------------------------%
% 6. COMPUTE AND DISPLAY THE LASER MEASUREMENTS
%----------------------------------------------------------------------%
%  6.1. SIMULATES A LASER SENSOR MEASUREMENT FROM THE INITIAL POSE
NOISE = 1;
[s_full,s] = my_laser.sense_using_laser(x, NOISE);
%  6.2. DISPLAY THE LASER BEAMS
my_laser.display_laser(x, s, my_grid.h_laser)
%  6.3. DISPLAY THE LASER OBSERVATIONS VIEWED FROM THE ROBOT
my_laser.create_laser_figure
my_laser.display_laser_end_points(s_full)
%  6.4. PROBABILITY OF THE IDEAL POSE
q = my_rfinder.likelihood_field_range_finder_model(s, x);
fprintf('The probability p(z/x) for the correct pose: %e \n', q)


%----------------------------------------------------------------------%
% 7. COMPUTE THE LIKELIHOOD OF THE MEASUREMENTS FOR SOME TEST POSES
%----------------------------------------------------------------------%
%  7.1. SET THE TEST POSES
AREA = 0.3;
NN = 50;
xx_test = linspace(x(1)-AREA, x(1)+AREA, NN);
yy_test = linspace(x(2)-AREA, x(2)+AREA, NN);
theta_test = x(3);
%  7.2. SET MESHGRID
[XXX, YYY] = meshgrid(xx_test, yy_test);
Xp = [XXX(:)  YYY(:)  ones(NN*NN,1)*theta_test];
%  7.3. COMPUTE PROBABILITIES
%   -> USING 'KNOWN' DISTANCES
tic
my_rfinder.set_distances_flag('KNOWN')
Q = my_rfinder.likelihood_field_range_finder_model(s, Xp);
Q = reshape(Q, size(XXX));
T = toc;
disp('---')
fprintf('With KNOWN min distances \n')
fprintf('The time to compute p(z|x) for the full grid: %2.4f(ms) \n', T*1000)
fprintf('The time to compute p(z|x) for the each grid - robot: %2.4f(ms) \n', T*1000/(NN*NN))
tic
%   -> USING 'UNKNOWN' DISTANCES
my_rfinder.set_distances_flag('UNKNOWN')
Q2 = my_rfinder.likelihood_field_range_finder_model(s, Xp);
Q2 = reshape(Q2, size(XXX));
T = toc;
disp('---')
fprintf('With UNKNOWN min distances \n')
fprintf('The time to compute p(z|x) for the full grid: %2.4f(ms) \n', T*1000)
fprintf('The time to compute p(z|x) for the each grid - robot: %2.4f(ms) \n', T*1000/(NN*NN))
%   -> CHECK RESULTS - NOTICE THAT IN BOTH CASES WE USED my_grid.set_maximum_distance(0.4)
E = Q - Q2;
disp('----')
if(any(any(E))==1)
    disp('min_distance ERROR')
else
    disp('min_distance OK')
end

%  7.4. PLOT THE LIKELIHOOD FUNCTION
figure('Name', 'Likelihood funtion p(z|x)')
surf(XXX,YYY, Q)
hold on
xlabel('x(m)')
ylabel('y(m)')
zlabel('p(z|x,m)')
axis([xx_test(1)  xx_test(end)  yy_test(1)  yy_test(end) 0  max(max(Q))])
%  7.5. PLOT THE ROBOT
plot_robot3(x, 0.05, max(max(Q)),'m')


%----------------------------------------------------------------------%
% 8. TO REDUCE OVERCONFIDENCE(pc. 183)
%----------------------------------------------------------------------%
%  8.1. COMPUTE THE A BETTER VERSION OF p(z|x)
if(MAPPER==1)  % According to ROS
    ALPHA  = 0.35;
    log_PZ = Q.^ALPHA;
else
    % For localization
    ALPHA  = 0.50;
    log_PZ = Q.^ALPHA;
end
fprintf('The probability p(z/x)^alpha for the correct pose: %e \n', q^ALPHA)
%  8.2. PLOT THE LOG
figure('Name', 'Likelihood funtion p(z|x)^alpha')
surf(XXX,YYY, log_PZ)
hold on
xlabel('x(m)')
ylabel('y(m)')
zlabel('p(z|x,m)^a')
axis([xx_test(1)  xx_test(end)  yy_test(1)  yy_test(end) 0  max(max(log_PZ))])
%  8.3. PLOT THE ROBOT
plot_robot3(x, 0.05, max(max(log_PZ)),'m')

% print -depsc figure.eps