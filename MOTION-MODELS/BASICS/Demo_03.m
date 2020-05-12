% Demo_03: Plot of a real robot using the function "display_vehicle"
%  Author: Ivan A. Calle Flores
clc; clearvars; close all
NAME_ROBOT = 'R2D2-R1';   % 'R2D2-00', 'R2D2-01', 'R2D2-R1', 'R2D2-R2'

%----------------------------------------------------------------------%
% 1. FIGURE CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. CREATE THE MAIN WINDOW
%   -> Create a figure and its handle
fig1 = figure;    
%   -> Set the position relative to the screen and the size of the figure,
%      the title of the figure, and the background color
set(fig1,'position',[10 50 500 500],'Name','R2D2-00 ROBOT',...
    'color',[211 208 200]/255);
%   -> Set additional properties
ax = gca;
set(ax, 'Box', 'on')
axis square
hold on
grid on
%  1.2. SET HANDLES TO PLOT THE ROBOT
if(strcmp(NAME_ROBOT,'R2D2-00')==1  ||  strcmp(NAME_ROBOT,'R2D2-01')==1)
    %   -> Matrix to save the handles
    handle_car = zeros(1,6);
    %   -> Handles to fill polygons "2 wheels + 1 box + 1 small sheel"
    for f=1:4
        handle_car(1,f) = fill(1, 1,'w');
    end
    %    -> Handles to draw lines(local reference frame)
    handle_car(1,5) = line(0,0,'LineWidth', 3);
    handle_car(1,6) = line(0,0,'LineWidth', 3);
    
elseif(strcmp(NAME_ROBOT, 'R2D2-R1')==1 || strcmp(NAME_ROBOT, 'R2D2-R2')==1)
    %   -> Matrix to save the handles
    handle_car = zeros(1,6);
    %   -> Handles to fill polygons "2 boxes + 2 wheels"
    for f=1:4
        handle_car(1,f) = fill(1, 1,'w');
    end
    %   -> Handles to draw lines(local reference frame)
    handle_car(1,5) = line(0,0,'LineWidth', 3);
    handle_car(1,6) = line(0,0,'LineWidth', 3);
end


%----------------------------------------------------------------------%
% 2. CREATE AN OBJECT "my_robot" OF THE CLASS "robot"
%----------------------------------------------------------------------%
%  2.1. CREATE AN OBJECT
my_robot = robot();
%  2.2. SET ROBOT
my_robot.set_robot(NAME_ROBOT)
fprintf('NAME_ROBOT: %s\n', my_robot.NAME_ROBOT)


%----------------------------------------------------------------------%
% 3. PLOT THE POSE OF THE ROBOT
%----------------------------------------------------------------------%
%  3.1. SET THE POSE
xx = 0.0;
yy = 0.0;
theta = 0*pi/4;
x = [xx; yy; theta];
my_robot.set_pose(x);
%  3.2. DISPLAY THE ROBOT
my_robot.display_vehicle(handle_car);
axis([-0.5  0.5  -0.5  0.5])
%  3.3. DISPLAY THE EQUIVALENT POSE
my_robot.plot_robot();
