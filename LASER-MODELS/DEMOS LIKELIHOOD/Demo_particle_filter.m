% Demo_particle_filter
%  Author: Ivan A. Calle Flores
clc; clearvars; close all

%----------------------------------------------------------------------%
% MAIN VARIABLES
%----------------------------------------------------------------------%
% GRID MAP
name_map  = 'map_01';
grid_file = 'map_01e.yaml';
path = '/home/ivan/IVAN/R2D2/TOOLBOX/MAPS/';
distances_file = [path grid_file(1:end-5) '_min_distances.mat'];
% ROBOT
MOTION_MODEL = 'STANDARD2';  % 'STANDARD','STANDARD2','IMPROVED'
NAME_ROBOT   = 'R2D2-00';
LEVEL_NOISE  = 3;
M = 200;           % Number of particles


%----------------------------------------------------------------------%
% 1. ENVIRONMENT INITIALIZATION
%----------------------------------------------------------------------%
%  1.1. CREATE AN OBJECT 'my_robot'
my_robot = robot(NAME_ROBOT);
my_robot.set_motion_model(MOTION_MODEL);
my_robot.set_level_of_noise(LEVEL_NOISE);
%  1.2. CREATE AN OBJECT "my_laser" WITH ITS ASSOCIATED MAP
my_laser = laser([path name_map]);
%  1.3. CREATE AN OBJECT "my_grid" WITH THE GRID MAP
my_grid = occ_grid(grid_file);
my_grid.set_robot(NAME_ROBOT);
my_grid.set_grid_map_min_distance(distances_file);
%  1.4. CREATE AN OBJECT "my_rfinder" WITH ITS ASSOCIATED GRID MAP
my_rfinder = rfinder();
my_rfinder.set_occ_grid(my_grid)
%my_rfinder.set_distances_flag('UNKNOWN')


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
KK = 30;
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
zhit  = 0.8;
zrand = 0.04;
zmax  = 0.16; 
sigma_hit = 0.1; 
my_rfinder.set_likelihood_field_parameters(zhit, zrand, zmax, sigma_hit);
%  4.2. LASER PARAMETERS
my_rfinder.set_theta_laser_LIKELIHOOD(THETA_LASER_SELECTED);
my_rfinder.set_laser_local_pose([xk, yk, thk]);
my_rfinder.set_max_range(my_laser.max_range);


%----------------------------------------------------------------------%
% 5. INITIAL ROBOT CONFIGURATION
%----------------------------------------------------------------------%
%  5.1 SET THE INITIAL POSE
xx = 8; 
yy = 7;
theta = 4*pi/8;
x = [xx; yy; theta];
my_robot.set_pose(x);
%  5.2. DISPLAY THE ROBOT USING THE SPECIFIED HANDLES
my_robot.display_vehicle(my_grid.h_vehicle);
%my_grid.display_robot(x)


%----------------------------------------------------------------------%
% 6. INITIALIZE PARTICLE SET
%----------------------------------------------------------------------%
%  6.1. SET AREA OF SAMPLING
WINDOWS_SIZE = 0.1;
xx_MIN    = x(1) - WINDOWS_SIZE;
xx_MAX    = x(1) + WINDOWS_SIZE;
yy_MIN    = x(2) - WINDOWS_SIZE;
yy_MAX    = x(2) + WINDOWS_SIZE;
theta_MIN = x(3) - pi/6;
theta_MAX = x(3) + pi/6;
%  6.2. SAMPLE POSES IN THE DEFINED AREA
%   -> Get samples from a "uniform" distribution
XX    =    xx_MIN + (xx_MAX-xx_MIN)*rand(M,1);
YY    =    yy_MIN + (yy_MAX-yy_MIN)*rand(M,1);
THETA = theta_MIN + (theta_MAX-theta_MIN)*rand(M,1);
THETA = pi_to_pi(THETA);
%   -> Save particles (each row is a pose)
X = [XX YY THETA];
%  6.4. PLOT PARTICLES
h_PARTICLES = plot(X(:,1), X(:,2),'b.','markersize',8);
%  6.5. COMPUTE GAUSSIAN APPROXIMATION
[mu, P] = compute_gaussian_from_samples(X);
ellipse_points = sigma_ellipse(mu(1:2), P(1:2,1:2), 1);
h_GAUSSIAN = plot(ellipse_points(1,:), ellipse_points(2,:),'m','LineWidth',2);


%----------------------------------------------------------------------%
% 7. MAIN CODE - LOCALIZATION
%----------------------------------------------------------------------%
STEPS = 50;
alpha = my_robot.alpha_VELOCITY;
dt = 0.2;
disp('Presiona una tecla para continuar')
pause()
TT = 0;
for i=1:STEPS
    fprintf('STEP: %d\n',i)
    
    %-------------------------------------------------------------%
    % A. VELOCITY COMMANDS
    %-------------------------------------------------------------%
    v = 0.25;           % Translational velocity (m/s)
	w = pi/8;              % Rotational velocity (rad/s)
    u_VELOCITY = [v; w];
    
    %-------------------------------------------------------------%
    % B. NOISE-FREE MOTION
    %-------------------------------------------------------------%
    x = my_robot.noise_free_motion_model_velocity(u_VELOCITY,dt);
    my_robot.display_vehicle(my_grid.h_vehicle);
    
    %-------------------------------------------------------------%
    % C. COMPUTE AND DISPLAY THE LASER MEASUREMENTS
    %-------------------------------------------------------------%
    %  GET MEASUREMENTS
    NOISE = 1;
    [s_full, s] = my_laser.sense_using_laser(x, NOISE);
    %  DISPLAY RESULTS
    my_laser.display_laser(x, s, my_grid.h_laser)
    my_laser.display_laser_end_points(s_full);
    
    %-------------------------------------------------------------%
    % D. PARTICLE FILTER
    %-------------------------------------------------------------%
    %  PREDICTION STEP
    tt = tic;
    X = sample_motion_model_velocity(X, u_VELOCITY, dt,...
            alpha, MOTION_MODEL);
    TT = TT + toc(tt);
    %  CORRECTION STEP
    %tt = tic;
    q = my_rfinder.likelihood_field_range_finder_model(s, X, 1);
    q = q.^0.35;
    %TT = TT + toc(tt);
    %  RESAMPLING
    [keep, Neff] = stratified_resample(q);
    X = X(keep,:);
    set(h_PARTICLES, 'xdata', X(:,1), 'ydata', X(:,2));
    %  COMPUTE GAUSSIAN APPROXIMATION
    [mu, P] = compute_gaussian_from_samples(X);
    ellipse_points = sigma_ellipse(mu(1:2), P(1:2,1:2), 1);
    set(h_GAUSSIAN, 'xdata', ellipse_points(1,:), 'ydata', ellipse_points(2,:));
    
    %-------------------------------------------------------------%
    % E. PAUSE
    %-------------------------------------------------------------%
    pause(0.05)
end
TT = TT/STEPS;
fprintf('time(ms): %2.4f\n', TT*1000)

% print -depsc figure.eps