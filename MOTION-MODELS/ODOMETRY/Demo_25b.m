% Demo_25b: Similar to Demo_25 but more real
%  Author: Ivan A. Calle Flores
clc; clearvars; close all
NAME_ROBOT   = 'R2D2-00';   % 'R2D2-00', 'R2D2-01', 'R2D2-R1', 'R2D2-R2'
MOTION_MODEL = 'STANDARD';  % 'STANDARD', 'STANDARD2', 'IMPROVED', 'GAUSSIAN'
LEVEL_NOISE  = 8;
DTRANS = [0.1, 0.25, 0.5, 1.0];
M = 5000;

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. CREATE AN OBJECT 'my_robot'
my_robot = robot(NAME_ROBOT);
my_robot.set_motion_model(MOTION_MODEL)
my_robot.set_level_of_noise(LEVEL_NOISE);
%  1.2. CREATE OBJECTS FOR THE SAMPLES
my_robots(1,M) = robot();
for m = 1:M
    my_robots(m).set_robot(NAME_ROBOT)
    my_robots(m).set_motion_model(MOTION_MODEL)
    my_robots(m).set_level_of_noise(LEVEL_NOISE)
end


%----------------------------------------------------------------------%
% 2. FIGURE CONFIGURATION
%----------------------------------------------------------------------%
%  2.1. CREATE THE MAIN WINDOW
fig1 = figure;    
set(fig1,'Name','Analize dt in odometry model',...
    'position',[10 50 400 800],'color',[211 208 200]/255);
%  2.2. CREATE AXES TO PLOT THE RESULTS
N = length(DTRANS);
H = zeros(N,1);
for k=1:N
    % -> Create axes
    H(k) = subplot(N,1,k);
    % -> Set atributes of the axes
    axis(H(k), [-0.3  1.3  -0.5  0.5])
    hold on
    grid on
    set(gca, 'box', 'on');
    ylabel('y(m)')
    % -> Set title
    dtrans = DTRANS(k);
    title(['\delta_{trans}=' num2str(dtrans)])
end
xlabel('x(m)')


%----------------------------------------------------------------------%
% 3. INITIAL CONFIGURATION OF THE ROBOT
%----------------------------------------------------------------------%
%  3.1. INITIAL POSE OF THE ROBOT
xx = 0.0;
yy = 0.0;
theta = 0*pi/8;
x = [xx; yy; theta];
%  3.2. ODOMETRY POSE
x_ODOMETRY = x;
my_robot.set_odometry_pose(x_ODOMETRY);


%----------------------------------------------------------------------%
% 4. MAIN LOOP
%----------------------------------------------------------------------%
for n=1:N
    disp('---')
    fprintf('DELTA: %2.2f\n',DTRANS(n))
    
    %----------------------------------------------------------------%
    % 4.1. INITIAL CONFIGURATION
    %----------------------------------------------------------------%
    %  A. GET DELTA OF TRANSLATION
    dtrans = DTRANS(n);
    %  B. GET AXES TO PLOT RESULTS
    subplot(H(n));
    
    %-----------------------------------------------------------------%
    % 4.2. SET INITIAL CONFIGURATION
    %-----------------------------------------------------------------%
    my_robot.set_pose(x);
    my_robot.plot_robot('b');
    for i=1:M
       my_robots(i).set_pose(x); 
    end
    
    %-----------------------------------------------------------------%
    % 4.3. SET ODOMETRY CONTROLS
    %-----------------------------------------------------------------%
    drot1 = 0;
    drot2 = 0*pi/8;
    u_ODOMETRY = [drot1; dtrans; drot2];
    fprintf('  u_ODOMETRY = [%2.4f, %2.4f, %2.4f]\n', u_ODOMETRY);
    
    %-----------------------------------------------------------------%
    % 4.4. MOTION CONFIGURATION
    %-----------------------------------------------------------------%
    DIST = 1.0;
    STEPS = DIST/(dtrans);
    STEPS = ceil(STEPS);     % To move 1 meter
    
    %-----------------------------------------------------------------%
    % 4.5. LOOP OF MOTION
    %-----------------------------------------------------------------%
    samples = zeros(M,3);       % Matrix to save samples
    for i=1:STEPS
        %  NOISE-FREE MOTION
        my_robot.noise_free_motion_model_odometry(u_ODOMETRY);
        %  GET SAMPLES FROM THE MOTION MODEL
        for m = 1:M
            samples(m,:) = my_robots(m).sample_motion_model_odometry(...
                u_ODOMETRY);
        end
    end
    
    %-----------------------------------------------------------------%
    % 4.6. COMPUTE STATISTICAL PARAMETERS OF THE TRUE SAMPLES
    %-----------------------------------------------------------------%
    %  A. COMPUTE THE MEAN AND VARIANCE ON THE POSITION
    [mu_samples, P_samples] = compute_gaussian_from_samples(samples);
    %  B. DISPLAY STATISTICAL RESULTS
    display_statistical_parameters(mu_samples,P_samples)
    
    %-----------------------------------------------------------------%
    % 4.7. PLOT RESULTS
    %-----------------------------------------------------------------%
    %  A. PLOT SAMPLES
    xx = samples(:,1);
    yy = samples(:,2); 
    plot(xx, yy,'.m')
    %  B. PLOT THE IDEAL ROBOT
    my_robot.plot_robot('k')
    %  C. PLOT THE GAUSSIAN APROXIMATION
    ellipse_points = sigma_ellipse(mu_samples(1:2), P_samples(1:2,1:2), 1);
    plot(ellipse_points(1,:), ellipse_points(2,:), 'C','LineWidth',2)
    ellipse_points = sigma_ellipse(mu_samples(1:2), P_samples(1:2,1:2), 2);
    plot(ellipse_points(1,:), ellipse_points(2,:), 'C','LineWidth',2)
    
    
    %-----------------------------------------------------------------%
    pause(0.1)
end

% set size of figure's "drawing" area on screen
set(gcf, 'Units','centimeters', 'Position',[0 1 10 20])
%# set size on printed paper
set(gcf, 'PaperPositionMode','auto')


% print -depsc 02_standard_odometry_samples.eps
% print -depsc 04_improved_odometry_samples.eps