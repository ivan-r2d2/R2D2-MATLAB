% Demo_71: Analize sampling and density evaluation times (M) - PAPER
%  Author: Ivan A. Calle Flores
clc; clearvars; close all
LEVEL_NOISE = 4;
MOTION_MODEL = 'IMPROVED';  %'STANDARD', 'STANDARD2', 'IMPROVED', 'GAUSSIAN'
NAME_ROBOT   = 'R2D2-00';
M = [5, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000,...
    1200, 1400, 1600, 1800, 2000];

%----------------------------------------------------------------------%
% 1. ENVIRONMENT INITIALIZATION
%----------------------------------------------------------------------%
%  1.1. CREATE A ROBOT
my_robot = robot();
my_robot.set_robot(NAME_ROBOT)
my_robot.set_motion_model(MOTION_MODEL)
my_robot.set_level_of_noise(LEVEL_NOISE)
%  1.2. CREATE AN IDEAL ROBOT
ideal_robot = robot; 
ideal_robot.set_robot(NAME_ROBOT)


%----------------------------------------------------------------------%
% 2. FIGURE CONFIGURATION
%----------------------------------------------------------------------%
%  2.1. CREATE THE MAIN WINDOW
fig1 = figure;    
set(fig1,'Name','Sampling from the velocity motion model',...
    'position',[10 50 800 500]);
grid on
hold on
xlabel('x(m)')
ylabel('y(m)')
set(gca, 'box', 'on');
axis([-0.5  1.0  -0.5  0.5])
axis square


%----------------------------------------------------------------------%
% 3. INITIAL CONFIGURATION OF THE ROBOT
%----------------------------------------------------------------------%
%  3.1. INITIAL POSE OF THE ROBOT
xx = 0.0; 
yy = 0.0;
theta = -0*pi/8;
x = [xx; yy; theta];
my_robot.set_pose(x);
ideal_robot.set_pose(x);
%  3.2. SET ODOMETRY POSE
x_ODOMETRY = x;
my_robot.set_odometry_pose(x_ODOMETRY);
%  3.3. PLOT POSE
my_robot.plot_robot()
%  3.4. DELTA OF TIME
dt = 1.25;


%----------------------------------------------------------------------%
% 4. COMPUTE THE MOTION
%----------------------------------------------------------------------%
%  4.1. SET VELOCITIES
v = 0.5;           % Translational velocity (m/s)
w = pi/6;              % Rotational velocity (rad/s)
u = [v; w];
%  4.2 SIMULATE THE ENCODERS FROM THE COMMANDED VELOCITIES
[nrev1, nrev2] = my_robot.simulate_encoders(u,dt);
xp_ODOMETRY = my_robot.compute_pose_from_encoder(nrev1, nrev2);
u_ODOMETRY  = compute_odometry_controls(x_ODOMETRY, xp_ODOMETRY);
fprintf('  u_ODOMETRY = [%2.4f, %2.4f, %2.4f]\n', u_ODOMETRY);
%  4.3. COMPUTE AND PLOT THE FINAL "noise-free" POSE
xp = ideal_robot.noise_free_motion_model_odometry(u_ODOMETRY);
ideal_robot.plot_robot('-k')
alpha = my_robot.alpha_ODOMETRY;
%  4.4. MAIN CODE
TT_SAMPLING     = zeros(length(M),1);
TT_SAMPLING_STD = zeros(length(M),1);
TT_DENSITY      = zeros(length(M),1);
TT_DENSITY_STD  = zeros(length(M),1);
for i=1:length(M)
    % MESSAGES
    disp('---')
    fprintf('M: %d\n', M(i))
    
    % ALLOCATE MEMORY FOR SAMPLES
    X = zeros(M(i),3);
    
    % LOOP
    REP = 10;
    T_SAMPLING = zeros(REP,1);
    T_DENSITY  = zeros(REP,1);
    for n=1:REP
        switch(MOTION_MODEL)
            case {'STANDARD', 'IMPROVED'}
                % Sampling
                T1 = tic;
                Xp = sample_motion_model_odometry(X, u_ODOMETRY, alpha,...
                    MOTION_MODEL);
                t = toc(T1)*1000;
                T_SAMPLING(n) = t;
                % Density p(x'|x,u)
                T1 = tic;
                PM = motion_model_odometry(Xp, u_ODOMETRY, x, alpha,...
                    MOTION_MODEL);
                t = toc(T1)*1000;
                T_DENSITY(n) = t;
                
            case 'GAUSSIAN'
                % Sampling
                T1 = tic;
                [mu, R] = my_robot.compute_gaussian_model_odometry(u_ODOMETRY);
                Xp      = mvnrnd(mu,R,M(i));
                Xp(:,3) = pi_to_pi(Xp(:,3));
                t = toc(T1)*1000;
                T_SAMPLING(n) = t;
                % Density p(x'|x,u)
                T1 = tic;
                PM = gauss_ND_robot(Xp,mu,R);        
                t = toc(T1)*1000;
                T_DENSITY(n) = t;
        end
    end
    
    % GET MEAN AND STD OF THE EXECUIION TIMES
    sampling_time     = mean(T_SAMPLING);
    sampling_time_std = std(T_SAMPLING);
    density_time      = mean(T_DENSITY);
    density_time_std  = std(T_DENSITY);
    fprintf('  Sampling time-mean(ms): %2.4f\n', sampling_time )
    fprintf('  Sampling time-std(ms): %2.4f\n', sampling_time_std )
    fprintf('  Density time-mean(ms): %2.4f\n', density_time )
    fprintf('  Density time-std(ms): %2.4f\n', density_time_std )
    
    % SAVE TIMES
    TT_SAMPLING(i)     = sampling_time;
    TT_SAMPLING_STD(i) = sampling_time_std;
    TT_DENSITY(i)      = density_time;
    TT_DENSITY_STD(i)  = density_time_std;
end
%  4.4. DISPLAY SAMPLES
xx = Xp(:,1);
yy = Xp(:,2); 
plot(xx, yy,'.m')


%----------------------------------------------------------------------%
% 5. COMPUTE STATISTICAL PARAMETERS OF THE MOTION
%----------------------------------------------------------------------%
%  5.1. COMPUTE THE MEAN AND VARIANCE ON THE POSITION
[mu, P] = compute_gaussian_from_samples(Xp, 0);
%  5.2. DISPLAY STATISTICAL RESULTS RESULTS
display_statistical_parameters(mu,P)
%  5.3. PLOT GAUSSIAN APROXIMATION
ellipse_points = sigma_ellipse(mu(1:2), P(1:2,1:2), 1);
plot(ellipse_points(1,:),ellipse_points(2,:),'c', 'LineWidth',2); 
ellipse_points = sigma_ellipse(mu(1:2), P(1:2,1:2), 2);
plot(ellipse_points(1,:),ellipse_points(2,:),'c','LineWidth',2);  
%  5.4. PLOT THE HISTOGRAM OF THE ANGLES
thetas = Xp(:,3);
display_hist_angles(thetas)


%----------------------------------------------------------------------%
% 5. DISCARD THE INITIAL RESULT FOR warm up REASONS
%----------------------------------------------------------------------%
MM = M(2:end);
TT_SAMPLING     = TT_SAMPLING(2:end,1);
TT_SAMPLING_STD = TT_SAMPLING_STD(2:end,1);
TT_DENSITY      = TT_DENSITY(2:end,1);
TT_DENSITY_STD  = TT_DENSITY_STD(2:end,1);


%----------------------------------------------------------------------%
% 6. PLOT RESULTS
%----------------------------------------------------------------------%
fig1 = figure;
set(fig1,'Name','Sampling execution time',...
    'position',[10 50 800 500],'color',[211 208 200]/255);
%  1.2. SET ITS PROPERTIES
set(gca, 'Box', 'on')
hold on
grid on
errorbar(MM, TT_SAMPLING, TT_SAMPLING_STD)
errorbar(MM, TT_DENSITY, TT_DENSITY_STD)
axis([0 MM(end)+100  0  max(TT_DENSITY)+0.5])
legend('sampling', 'density', 'Location','northwest','FontSize',14)
xlabel('M')
ylabel('Time(ms)')