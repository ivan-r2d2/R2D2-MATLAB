function plot_robot3(x, B, z, color)
% PLOT_ROBOT3 Plots the actual pose of a robot in the actual figure
%
% INPUTS
%       x: Pose of the robot "[x;y;theta]"
%       B: Distance from each wheel to the center point
%       z: Height
%   color: Color of the robot

%----------------------------------------------------------------------%
% 1. ARGUMENTS' CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK MAIN ARGUMENTS
if nargin < 3
    error('PLOT_ROBOT3: Insufficient number of arguments')
end
%  1.2. CHECK DIMENSION OF THE POSE
if length(x) ~= 3
    error('PLOT_ROBOT3: Pose must be of length 3')
end        
%  1.3. SET DEFAULT COLOR
if nargin < 4
    color = '-b';
end
%  1.4. SET DEFAULT NUMBER OF POINTS TO PLOT THE ROBOT
NN = 24;

%----------------------------------------------------------------------%
% 2. GET THE POSE OF THE ROBOT
%----------------------------------------------------------------------%
%  2.1. GET THE LOCATION
xx = x(1); 
yy = x(2);
%  2.2. GET THE ORIENTATION
theta = x(3);
if(theta > pi ||theta <= -pi)
    error('PLOT_ROBOT3: The angle theta of the robot is not in the range.')
end

%----------------------------------------------------------------------%
% 3. PLOT THE ROBOT
%----------------------------------------------------------------------%
%  3.1. COMPUTE THE POINTS OF THE CIRCLE
ang = linspace(0,2*pi,NN)';
x_circle = xx*ones(NN,1) + B*cos(ang);         % x-components
y_circle = yy*ones(NN,1) + B*sin(ang);         % y_components
z_circle = z*ones(NN,1);
%  3.2. PLOT THE CIRCLE
plot3(x_circle,y_circle, z_circle, color,'lineWidth',2)
hold on
%  3.3. PLOT THE LINE THAT REPRESENT THE ORIENTATION
xf = xx + B*cos(theta);
yf = yy + B*sin(theta);
plot3([xx  xf], [yy  yf], [z z], color, 'lineWidth',3)

end