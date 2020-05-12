function display_robot(x, B, h_robot, color, NN)
% DISPLAY_ROBOT Plots the actual pose of a robot using handles
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
if(nargin < 3)
    error('DISPLAY_ROBOT: Insufficient number of arguments')
end
%  1.2. CHECK DIMENSION OF THE POSE
if(length(x) ~= 3)
    error('DISPLAY_ROBOT: Pose must be of length 3')
end
%  1.3. CHECK DIMENSION OF THE HANDLE
if(length(h_robot) ~= 2)
    error('DISPLAY_ROBOT: Handles must be of length 2')
end
%  1.4. SET DEFAULT COLOR
if(nargin < 4)
    color = 'b';
end
%  1.5. SET DEFAULT NUMBER OF POINTS TO PLOT THE ROBOT
if(nargin < 5)
    NN = 24;
end

%----------------------------------------------------------------------%
% 2. GET THE POSE OF THE ROBOT
%----------------------------------------------------------------------%
%  2.1. GET THE LOCATION
xx = x(1); 
yy = x(2);
%  2.2. GET THE ORIENTATION
theta = x(3);
if(theta > pi ||theta <= -pi)
    error('DISPLAY_ROBOT: The angle theta of the robot is not in the range.')
end

%----------------------------------------------------------------------%
% 3. COMPUTE REPRESENTATIVE POINTS OF THE ROBOT
%----------------------------------------------------------------------%
%  3.1. COMPUTE THE POINTS OF THE CIRCLE
ang = linspace(0,2*pi,NN)';
x_circle = xx*ones(NN,1) + B*cos(ang);         % x-components
y_circle = yy*ones(NN,1) + B*sin(ang);         % y_components
%  3.2. COMPUTE THE POINTS OF THE ORIENTATION
xf = xx + B*cos(theta);
yf = yy + B*sin(theta);
linex = [xx  xf];
liney = [yy  yf];

%----------------------------------------------------------------------%
% 4. DRAW THE ROBOT
%----------------------------------------------------------------------%
set(h_robot(1),'xdata',x_circle,'ydata',y_circle,'Color',color);
set(h_robot(2),'xdata',linex,'ydata',liney,'Color',color);

end