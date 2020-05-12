% Demo_02: Plot of a robot's pose using "display_robot" to do animations
%  Author: Ivan A. Calle Flores
clc; clearvars; close all

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. CREATE AN OBJECT "my_robot"
my_robot = robot();


%----------------------------------------------------------------------%
% 2. SET PARAMETERS
%----------------------------------------------------------------------%
%  2.1. MECHANICAL PARAMETERS
D = 0.1;        % Diameter of the wheels(m)
B = 0.15;       % Distance from each wheel to the center point(m)
my_robot.set_mechanical_parameters(D, B)
%  2.2. INITIAL POSE
xx = 1;                 % x-position(m)
yy = 1;                 % y-position(m)
theta = pi/4;           % Heading direction(rad)
x = [xx;yy;theta];      % Pose
my_robot.set_pose(x)


%----------------------------------------------------------------------%
% 3. PLOT THE ROBOT
%----------------------------------------------------------------------%
%  3.1. CREATE A FIGURE
fig = figure;
set(gca, 'box', 'on')
%  3.2. SET HANDLES TO PLOT THE ROBOT
handle_robot = zeros(1,2);
handle_robot(1) = line(0,0,'LineWidth', 2);
handle_robot(2) = line(0,0,'LineWidth', 2);
%  3.3. CALL THE METHOD "display_robot"
my_robot.display_robot(handle_robot,'b')
axis([-1  2  -1  2])
grid on
pause(1.0)


%----------------------------------------------------------------------%
% 4. SOME ANIMATIONS
%----------------------------------------------------------------------%
%  4.1. FIRST MOTION
my_robot.set_pose([1;-0.5;0]);
my_robot.display_robot(handle_robot,'b')  
pause(1.0)
%  4.2. SECOND MOTION
my_robot.set_pose([-0.5;-0.5;pi]);
my_robot.display_robot(handle_robot,'b')
pause(1.0)