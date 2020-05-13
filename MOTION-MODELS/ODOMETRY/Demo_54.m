% Demo_54: Grid of points - FULL 3D
%  Author: Ivan A. Calle Flores
clc; clearvars; close all
NAME_ROBOT   = 'R2D2-00';   % 'R2D2-00', 'R2D2-01', 'R2D2-R1', 'R2D2-R2'
MOTION_MODEL = 'STANDARD2';  % 'STANDARD', 'STANDARD2', 'IMPROVED', 'GAUSSIAN'
LEVEL_NOISE  = 4;

%----------------------------------------------------------------------%
% 1. CREATE AN OBJECT OF THE CLASS "robot"
%----------------------------------------------------------------------%
my_robot = robot; 
my_robot.set_robot(NAME_ROBOT )
my_robot.set_motion_model(MOTION_MODEL)
my_robot.set_level_of_noise(LEVEL_NOISE);


%----------------------------------------------------------------------%
% 2. FIGURE CONFIGURATION
%----------------------------------------------------------------------%
fig1 = figure;    
set(fig1,'Name','p(xp | u, x) using the odometry model',...
    'position',[10 50 500 500],'color',[211 208 200]/255, 'Visible', 'Off');
hold on
xlabel('x(m)')
ylabel('y(m)')
set(gca, 'box', 'on');


%----------------------------------------------------------------------%
% 3. INITIAL CONFIGURATION OF THE ROBOT
%----------------------------------------------------------------------%
%  3.1. INITIAL POSE OF THE ROBOT
xx = 0.2;
yy = 0.4;
theta = 0*pi_to_pi(8*pi/8-0*pi/64);
x = [xx; yy; theta];
my_robot.set_pose(x);
%  3.2. SET ODOMETRY POSE
x_ODOMETRY = x;
my_robot.set_odometry_pose(x_ODOMETRY);


%----------------------------------------------------------------------%
% 4. CONTROLS CONFIGURATION 
%----------------------------------------------------------------------%
%  4.1. VELOCITY COMMANDS
v = -1*0.2;            % Translational velocity (m/s)
w = -1*pi/16;        % Rotational velocity (rad/s)
u = [v; w];
%  4.2. DELTA OF TIME
dt = 5.0;
%  4.3 SIMULATE THE ENCODERS FROM THE COMMANDED VELOCITIES
[nrev1, nrev2] = my_robot.simulate_encoders(u,dt);
%  4.4. COMPUTE ODOMETRY CONTROLS
xp_ODOMETRY = my_robot.compute_pose_from_encoder(nrev1, nrev2);
u_ODOMETRY = compute_odometry_controls(x_ODOMETRY, xp_ODOMETRY);
fprintf('  u_ODOMETRY = [%2.4f, %2.4f, %2.4f]\n', u_ODOMETRY);


%----------------------------------------------------------------------%
% 5. COMPUTE "p(xp | u, x)" AT THE NOISE-FREE LAST POSE
%----------------------------------------------------------------------%
%  5.1. COMPUTE THE NEXT "noise-free" POSE USING THE ODOMETRY MODEL
xp1 = my_robot.noise_free_motion_model_odometry(u_ODOMETRY);
xp = xp1;
fprintf('\nThe final noise-free odometry pose of the movil robot is:\n')
my_robot.display_pose(1)
%  5.2. COMPUTE ITS PROBABILITY
my_robot.set_pose(x);
[p, drot1_p, dtrans_p, drot2_p, e_rot1, e_trans, e_rot2] = ...
    my_robot.motion_model_odometry(xp1, u_ODOMETRY);
fprintf('  p = %4.4f\n', p)
fprintf('  drot1_true = %2.4f  dtrans_true = %2.4f  drot2_true = %2.4f\n', ...
    drot1_p, dtrans_p, drot2_p)
fprintf('  e_rot1 = %2.4f  e_trans = %2.4f  e_rot2 = %2.4f\n', ...
    e_rot1, e_trans, e_rot2)


%----------------------------------------------------------------------%
% 6. COMPUTE "p(xp | u, x)" FOR A GRID OF POINTS
%----------------------------------------------------------------------%
%  6.1. COMPUTE AREA OF SEARCH
B = my_robot.B;
min_pos = min(x,xp);
XX_INIT = min_pos(1) - 2*B;
YY_INIT = min_pos(2) - 2*B;
max_pos = max(x,xp);
XX_END = max_pos(1) + 2*B;
YY_END = max_pos(2) + 2*B;
%  6.2. SET THE GRID OF POINTS AROUND "xp"
NN = 100;
x_test = linspace(XX_INIT, XX_END, NN);
y_test = linspace(YY_INIT, YY_END, NN);
%  6.3. SET THE ORIENTATION OF THE POINTS
theta_test = xp(3)-pi/4:pi/128:xp(3)+pi/4;      % Around "theta_p"
theta_test = pi_to_pi(theta_test);
%  6.3. COMPUTE PROBABILITIES
tic
[XXX, YYY, TTT] = meshgrid(x_test, y_test, theta_test);
Xp = [XXX(:)  YYY(:)  TTT(:)];
switch MOTION_MODEL
    case {'STANDARD','STANDARD2','IMPROVED'}
        a = motion_model_odometry(Xp, u_ODOMETRY, x,...
                my_robot.alpha_ODOMETRY, MOTION_MODEL);
    case 'GAUSSIAN'
        [mu, R] = my_robot.compute_gaussian_model_odometry(u_ODOMETRY);
        a = gauss_ND_robot(Xp,mu,R);
end
TIME = toc;
fprintf('Elapsed time: %2.4f\n',TIME)


%----------------------------------------------------------------------%
% 7. GET SAMPLES FROM "p(xp | u, x)"
%----------------------------------------------------------------------%
%  7.1. GET SAMPLES
N = 5000;
switch MOTION_MODEL
    case {'STANDARD','STANDARD2','IMPROVED'}
        X = ones(N,1)*(x');
        samples = sample_motion_model_odometry(X, u_ODOMETRY,...
            my_robot.alpha_ODOMETRY, MOTION_MODEL);
    case 'GAUSSIAN'
        [mu, R] = my_robot.compute_gaussian_model_odometry(u_ODOMETRY);
        samples      = mvnrnd(mu,R,N);
        samples(:,3) = pi_to_pi(samples(:,3));
end
%  7.2. PLOT THE SAMPLES(only the locations)
xx = samples(:,1);
yy = samples(:,2); 
%  7.3. PLOT RESULTS
fig2 = figure;
hold on
axis square
plot(xx, yy,'.k')
axis([x_test(1)  x_test(end)  y_test(1)  y_test(end)])
grid on
% Plot poses
my_robot.plot_robot('-r')
plot_robot(xp, my_robot.B, '-m')


%----------------------------------------------------------------------%
% 8. PLOT THE PROBABILITY "p(xp | u, x)" FOR THE GRID OF POINTS
%    USING AN IMAGE IN 2D
%----------------------------------------------------------------------%
%  8.1. ADD THE PROBABILITIES ALONG THE ANGLE THETA
p = reshape(a, size(XXX));  % First reshape
P = sum(p,3);
%  8.2. PLOT THE PROBABILITY MATRIX AS AN IMAGE
%   -> Get Figure 1 and make it visible
figure(fig1);
set(fig1, 'Visible', 'on');
%   -> Plot the probaility as an image
imagesc([x_test(1) x_test(end)], [y_test(1) y_test(end)], P)
colormap('gray')
colorbar
%   -> To align correctly the image
set(gca, 'Ydir', 'normal');
%   -> Additional properties
xlabel('x(m)')
ylabel('y(m)')
hold on
axis([x_test(1)  x_test(end)  y_test(1)  y_test(end)])
axis square
%  7.3. PLOT THE INITIAL POSE, THE ARC OF ROTATION, AND THE LAST POSE
%   -> Initial pose
plot_robot(x, my_robot.B, '-r')
%   -> Last "noise-free" pose
plot_robot(xp, my_robot.B, '-m')


%----------------------------------------------------------------------%
% 8. PLOT THE PROBABILITY "p(xp | u, x)" FOR THE GRID OF POINTS
%    USING AN SURFACE IN 3D
%----------------------------------------------------------------------%
%  8.1. PLOT THE PROBABILITY IN 3D
%   -> Create an image
fig3 = figure;
hold on
%   -> Plot the surface
surf(x_test, y_test, P)
axis([x_test(1)  x_test(end)  y_test(1)  y_test(end)  0  max(max(P))])
xlabel('x(m)')
ylabel('y(m)')
%   -> Additional properties
view(3)
grid on
%  8.2. PLOT THE FINAL POSE
plot_robot3(xp, 0.1, max(max(P)),'-r')