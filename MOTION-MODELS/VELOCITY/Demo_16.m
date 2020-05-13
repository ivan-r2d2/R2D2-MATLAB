% Demo_16: Analize 'dt' - PAPER
%  Author: Ivan A. Calle Flores
clc; clearvars; close all
NAME_ROBOT   = 'R2D2-00';   % 'R2D2-00', 'R2D2-01', 'R2D2-R1', 'R2D2-R2'
MOTION_MODEL = 'STANDARD';  % 'STANDARD', 'STANDARD2', 'IMPROVED', 'GAUSSIAN'
LEVEL_NOISE  = 4;
DT = [0.25, 0.5, 1.0, 2.5];
M = 5000;

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. CREATE AN OBJECT 'my_robot'
my_robot = robot; 
my_robot.set_robot(NAME_ROBOT )
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
set(fig1,'Name','p(xp | u, x) using the velocity model',...
    'position',[10 50 400 800],'color',[211 208 200]/255);
%  2.2. CREATE AXES TO PLOT THE RESULTS
N = length(DT);
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
    dt = DT(k);
    title(['\Deltat=' num2str(dt)])
end
xlabel('x(m)')


%----------------------------------------------------------------------%
% 3. INITIAL CONFIGURATION OF THE ROBOT
%----------------------------------------------------------------------%
xx = 0.0;
yy = 0.0;
theta = 0*pi/8;
x = [xx; yy; theta];


%----------------------------------------------------------------------%
% 4. CONTROLS CONFIGURATION 
%----------------------------------------------------------------------%
%  4.1. VELOCITY COMMANDS
v = 0.2;        % Translational velocity (m/s)
w = 0;          % Rotational velocity (rad/s)
u_VELOCITY = [v; w];


%----------------------------------------------------------------------%
% 5. MAIN LOOP
%----------------------------------------------------------------------%
for n=1:N
    fprintf('\nDELTA: %2.2f\n',DT(n))
    
    %----------------------------------------------------------------%
    % 5.1. INITIAL CONFIGURATION
    %----------------------------------------------------------------%
    %  A. GET DELTA OF TIME
    dt = DT(n);
    %  B. GET AXES TO PLOT RESULTS
    subplot(H(n));
    
    %-----------------------------------------------------------------%
    % 5.2. SET INITIAL CONFIGURATION
    %-----------------------------------------------------------------%
    my_robot.set_pose(x);
    my_robot.plot_robot('b');
    for i=1:M
       my_robots(i).set_pose(x); 
    end
    
    %-----------------------------------------------------------------%
    % 5.3. MOTION CONFIGURATION
    %-----------------------------------------------------------------%
    DIST = 1.0;
    STEPS = DIST/(v*dt);
    STEPS = ceil(STEPS);     % To move 1 meter
    
    %-----------------------------------------------------------------%
    % 5.4. LOOP OF MOTION
    %-----------------------------------------------------------------%
    samples = zeros(M,3);       % Matrix to save samples
    for i=1:STEPS
        %  NOISE-FREE MOTION
        my_robot.noise_free_motion_model_velocity(u_VELOCITY, dt);
        %  GET SAMPLES FROM THE MOTION MODEL
        for m = 1:M
            samples(m,:) = my_robots(m).sample_motion_model_velocity(...
                u_VELOCITY, dt);
        end
    end
    
    %-----------------------------------------------------------------%
    % 5.5. COMPUTE STATISTICAL PARAMETERS OF THE TRUE SAMPLES
    %-----------------------------------------------------------------%
    %  A. COMPUTE THE MEAN AND VARIANCE ON THE POSITION
    [mu_samples, P_samples] = compute_gaussian_from_samples(samples);
    %  B. DISPLAY STATISTICAL RESULTS
    display_statistical_parameters(mu_samples,P_samples)
    
    %-----------------------------------------------------------------%
    % 5.6. PLOT RESULTS
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


% print -depsc 01_standard_velocity_samples.eps