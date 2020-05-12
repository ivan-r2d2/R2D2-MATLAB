% Demo_01: Plot a robot's pose using "plot_robot"
%  Author: Ivan A. Calle Flores
clc; clearvars; close all

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. CREATE AN OBJECT "my_robot"
my_robot = robot();


%----------------------------------------------------------------------%
% 2. SET PARAMETERS OF THE ROBOT
%----------------------------------------------------------------------%
%  2.1.SET  MECHANICAL PARAMETERS
%   -> Set parameters
D = 0.1;        % Diameter of the wheels(m)
B = 0.15;       % Distance from each wheel to the center point(m)
%   -> Call method to set parameters
my_robot.set_mechanical_parameters(D, B)
%   -> Display the properties
disp('Diameter of the wheels(m)')
disp(my_robot.D)
disp('Distance from each wheel to the center point(m)')
disp(my_robot.B)
%  2.2. SET THE POSE
%   -> Set pose
xx = 1;                 % x-position(m)
yy = 1;                 % y-position(m)
theta = -4*pi/4;        % Heading direction(rad)
x = [xx; yy; theta];    % Pose
%   -> Call method to set the pose
my_robot.set_pose(x)
%   -> Call method to display the pose
my_robot.display_pose(1)


%----------------------------------------------------------------------%
% 3. PLOT THE ROBOT
%----------------------------------------------------------------------%
%  3.1. CREATE A FIGURE
fig = figure;
%  3.2. CALL THE METHOD "plot_robot"
my_robot.plot_robot('b')
axis([-1  2  -1  2])
xlabel('x(m)')
ylabel('y(m)')
grid on