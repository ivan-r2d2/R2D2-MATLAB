% Demo_27: Analize rotations
%  Author: Ivan A. Calle Flores
clc; clearvars; close all
NAME_ROBOT   = 'R2D2-00';   % 'R2D2-00', 'R2D2-01', 'R2D2-R1', 'R2D2-R2'
MOTION_MODEL = 'IMPROVED';  % 'STANDARD', 'STANDARD2', 'IMPROVED', 'GAUSSIAN'
LEVEL_NOISE  = 8;
M = 5000;
THETA = [0; pi/4; pi/2];

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
    'position',[10 50 300 800],'color',[211 208 200]/255);
%  2.2. CREATE AXES TO PLOT THE RESULTS
N = length(THETA);
H = zeros(N,1);
for k=1:N
    % -> Create axes
    H(k) = subplot(N,1,k);
    % -> Set atributes of the axes
    axis(H(k), [-0.2  0.2  -0.2  0.2])
    hold on
    grid on
    set(gca, 'box', 'on');
    ylabel('y(m)')
    axis square
    % -> Set title
    th = THETA(k);
    title(['$\bar{\theta}$=' num2str(th)],'interpreter','Latex')
end
xlabel('x(m)')


%----------------------------------------------------------------------%
% 3. INITIAL POSITION 
%----------------------------------------------------------------------%
xx = 0.0;
yy = 0.0;


%----------------------------------------------------------------------%
% 4. CONTROLS CONFIGURATION 
%----------------------------------------------------------------------%
%  4.1. VELOCITY COMMANDS
v = 0.000;        % (m/s)
w = +pi/10;
u_VELOCITY = [v; w];
dt = 5.0;


%----------------------------------------------------------------------%
% 5. MAIN LOOP
%----------------------------------------------------------------------%
for n=1:N
    %----------------------------------------------------------------%
    % 5.1 SET INITIAL CONFIGURATION
    %----------------------------------------------------------------%
    %  GET ANGLE
    theta = THETA(n);
    disp('---')
    %fprintf('\n theta: %2.2f\n', theta)
    %  SET POSE
    x = [xx; yy; theta];
    my_robot.set_pose(x);
    for i=1:M
       my_robots(i).set_pose(x); 
    end
    %  ODOMETRY POSE
    x_ODOMETRY = x;
    my_robot.set_odometry_pose(x_ODOMETRY);
    
    %-----------------------------------------------------------------%
    % 5.2. PLOT ROBOT
    %-----------------------------------------------------------------%
    subplot(H(n));
    my_robot.plot_robot('b');
    
    %-----------------------------------------------------------------%
    % 5.3. COMPUTE ODOMETRY CONTROLS
    %-----------------------------------------------------------------%
    [nrev1, nrev2] = my_robot.simulate_encoders(u_VELOCITY,dt);
    xp_ODOMETRY = my_robot.compute_pose_from_encoder(nrev1, nrev2, 1);
    u_ODOMETRY  = my_robot.u_ODOMETRY;
    fprintf('  u_ODOMETRY = [%2.4f, %2.4f, %2.4f]\n', u_ODOMETRY);
    %disp(xp_ODOMETRY')
    
    %-----------------------------------------------------------------%
    % 5.4. LOOP OF MOTION
    %-----------------------------------------------------------------%
    %  NOISE-FREE MOTION
    my_robot.noise_free_motion_model_odometry(u_ODOMETRY);
    %  GET SAMPLES FROM THE MOTION MODEL
    samples = zeros(M,3);
    for m = 1:M
        samples(m,:) = my_robots(m).sample_motion_model_odometry(...
            u_ODOMETRY);
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
    plot(samples(:,1), samples(:,2),'.m')
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
%set(gcf, 'Units','centimeters', 'Position',[0 1 10 20])
%# set size on printed paper
%set(gcf, 'PaperPositionMode','auto')


% print -depsc 05_standard_odometry_samples_rota.eps
% print -depsc 05_improved_odometry_samples_rotb.eps