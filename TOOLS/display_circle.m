function display_circle(x, B, h_robot, color, NN)
% DISPLAY_CIRCLE Plots the actual pose of a robot using handles
%
% INPUTS
%       x: Pose of the robot "[x;y;theta]"
%       B: Distance from each wheel to the center point
% h_robot: Handles to plot the robot
%   color: Color of the robot
%      NN: Number of points of the circle

%----------------------------------------------------------------------%
% 1. ARGUMENTS' CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK MAIN ARGUMENTS
if nargin < 3
    error('DISPLAY_CIRCLE: Insufficient number of arguments')
end
%  1.2. CHECK DIMENSION OF THE POSE
if length(x) ~= 2
    error('DISPLAY_CIRCLE: Pose must be of length 2')
end      
%  1.3. SET DEFAULT COLOR
if nargin < 4
    color = 'b';
end
%  1.4. SET DEFAULT NUMBER OF POINTS TO PLOT THE ROBOT
if nargin < 5
    NN = 24;
end

%----------------------------------------------------------------------%
% 2. GET THE LOCATION OF THE ROBOT
%----------------------------------------------------------------------%
xx = x(1); 
yy = x(2);

%----------------------------------------------------------------------%
% 3. COMPUTE REPRESENTATIVE POINTS OF THE CIRCLE
%----------------------------------------------------------------------%
ang = linspace(0,2*pi,NN)';
x_circle = xx*ones(NN,1) + B*cos(ang);         % x-components
y_circle = yy*ones(NN,1) + B*sin(ang);         % y_components

%----------------------------------------------------------------------%
% 4. DRAW THE ROBOT
%----------------------------------------------------------------------%
set(h_robot,'xdata', x_circle,'ydata',y_circle,'Color',color);


end