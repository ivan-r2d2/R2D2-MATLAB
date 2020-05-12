function plot_robot(x, B, color, NN)
% PLOT_ROBOT Plots the actual pose of a robot in the actual figure
%
% INPUTS
%      x: Pose of the robot "[x;y;theta]"
%      B: Distance from each wheel to the center point
%  color: Color of the robot
%     NN: Number of points of the circle
%
% NOTE: With this function you can plot the pose of any kind of robot
%       (differential, ackerman, etc)

%----------------------------------------------------------------------%
% 1. ARGUMENTS' CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK MAIN ARGUMENTS
if(nargin < 2)
    error('PLOT_ROBOT: Insufficient number of arguments')
end
%  1.2. CHECK DIMENSION OF THE POSE
if(length(x) ~= 3)
    error('PLOT_ROBOT: Pose must be of length 3')
end      
%  1.3. SET DEFAULT COLOR
if(nargin < 3)
    color = '-b';
end
%  1.4. SET DEFAULT NUMBER OF POINTS TO PLOT THE ROBOT
if(nargin < 4)
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
    error('PLOT_ROBOT: The angle is not in the range (-pi, pi].')
end

%----------------------------------------------------------------------%
% 3. PLOT THE ROBOT
%----------------------------------------------------------------------%
%  3.1. COMPUTE THE POINTS OF THE CIRCLE
ang = linspace(0,2*pi,NN)';
x_circle = xx*ones(NN,1) + B*cos(ang);         % x-components
y_circle = yy*ones(NN,1) + B*sin(ang);         % y_components
%  3.2. PLOT THE CIRCLE
plot(x_circle,y_circle, color,'lineWidth',2)
hold on
%  3.3. PLOT THE LINE THAT REPRESENT THE ORIENTATION
xf = xx + B*cos(theta);
yf = yy + B*sin(theta);
plot([xx  xf], [yy  yf], color, 'lineWidth',3)


end