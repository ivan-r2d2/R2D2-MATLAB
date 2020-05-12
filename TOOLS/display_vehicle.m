function handle_car = display_vehicle(pose, handle_car, robot)
% DISPLAY_VEHICLE Draws our real vehicle using the specified handlers
%
% OUTPUT
%  handle_car: The handles that we use to fill 2D polygons and draw lines
%
% INPUTS
%        pose: The actual pose to be drawn
%  handle_car: The handles to fill 2D polygons and draw lines
%       robot: Structure that contains the parameters of the robot

%----------------------------------------------------------------------%
% 1. ARGUMENTS CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK NECCESARY ARGUMENTS
if(nargin < 3)
    error('DISPLAY_VEHICLE: Insuficient number of parameters')
end
%  1.2. MAKE SURE THAT THE ARGUMENT "robot" IS A STRUCT
if(~isstruct(robot))
    error('DISPLAY_VEHICLE: The fourth argument must be a structure')
end
%  1.3. CHECK THE DIMENSION OF THE VECTOR OF HANDLES
if(length(handle_car) ~= 6)
    error('DISPLAY_VEHICLE: The handler matrix must be of size 6')
end


%----------------------------------------------------------------------%
% 2. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  2.1. GET THE MECHANICAL DIMENSIONS OF THE VEHICLE
Length = robot.Length;          % Length of the vehicle[m] - 0.4m
Width  = robot.Width ;          % Width of the vehicle[m] - 0.3m
Eje    = robot.Eje;             % Distance from the axis to the front
D      = robot.Diameter;        % Diameter of the wheels[m]
ww     = robot.WW;              % "Total Width"  of the wheels
%  2.2. GET THE POSITION AND ORIENTATION
position    = pose(1:2)';       % Position 
orientation = pose(3);          % Orientation


%----------------------------------------------------------------------%
% 3. COMPUTE THE COORDINATES OF EACH COMPONENT OF THE ROBOT
%----------------------------------------------------------------------%
%  3.1 PRINCIPAL BODY
%    -> Set the coordinates of the corners relative to the local coord.
x_car = [-(Length-Eje)  Eje  Eje  -(Length-Eje)];
y_car = Width*[ 0.5  0.5  -0.5  -0.5];
%   -> Convert these to polar coordinates
[ang, radio] = cart2pol(x_car,y_car);
%   -> Convert these back to cartesian coord. but with the new orientation
[x_car,y_car] = pol2cart(ang + orientation, radio);
%   -> Set the new positions of the corners
x_car = x_car + position(1);
y_car = y_car + position(2);

%  3.2. WHEEL NUMBER 1 - RIGHT ONE "motor 1"
%    -> Set the coordinates of the corners relative to the local coord.
x1_wheels = [-D/2   D/2   D/2   -D/2];
y1_wheels = [-0.5*Width -0.5*Width -(0.5*Width+ww) -(0.5*Width+ww)];
%    -> Convert these to polar coordinates
[ang, radio] = cart2pol(x1_wheels,y1_wheels);
%    -> Convert these back to cartesian coor but with the new orientation
[x1_wheels,y1_wheels] = pol2cart(ang + orientation, radio);
%    -> Set the new positions of the corners
x1_wheels = x1_wheels + position(1);
y1_wheels = y1_wheels + position(2);

%  3.3. WHEEL NUMBER 2  - LEFT ONE "motor 2"
%    -> Set the coordinates of the corners relative to the local coord.
x2_wheels = [-D/2   D/2   D/2   -D/2];
y2_wheels = [0.5*Width+ww   0.5*Width+ww   0.5*Width   0.5*Width];
%   -> Convert these to polar coordinates
[ang, radio] = cart2pol(x2_wheels,y2_wheels);
%   -> Convert these back to cartesian coor but with the new orientation
[x2_wheels,y2_wheels] = pol2cart(ang + orientation, radio);
%    -> Set the new positions of the corners
x2_wheels = x2_wheels + position(1);
y2_wheels = y2_wheels + position(2);

%   3.4. POSTERIOR WHEEL - Rueda loca
%   -> Set parameters of the wheel
d = Length - Eje - 0.05;    % Distance from the local frame
r = 0.02;                   % Radious of the wheel        
%    -> Set representative points of the circle(robot frame)
NN = 24;  
ang = linspace(0,2*pi,NN)';
x3_wheels = -d*ones(NN,1) + r*cos(ang);        % x-components
y3_wheels =  0.0*ones(NN,1) + r*sin(ang);      % y_components
%   -> Convert these to polar coordinates
[ang, radio] = cart2pol(x3_wheels,y3_wheels);
%   -> Convert these back to cartesian coor but with the new orientation
[x3_wheels,y3_wheels] = pol2cart(ang + orientation, radio);
%    -> Set the circle in the global frame
x3_wheels = x3_wheels + position(1);
y3_wheels = y3_wheels + position(2);


%----------------------------------------------------------------------%
% 4. SET THE AXIS OF THE ROBOT
%----------------------------------------------------------------------%
%  4.1 LINE 1 - Represent the x-axis of the local coordinate frame
%    -> Set the coordinates of the line
x_line1 = [-3/100  10/100];
y_line1 = [   0      0 ];
%   -> Convert these to polar coordinates
[ang, radio] = cart2pol(x_line1,y_line1);
%   -> Convert these back to cartesian coord. but with the new orientation
[x_line1,y_line1] = pol2cart(ang + orientation, radio);
%   -> Set the new location
x_line1 = x_line1 + position(1);
y_line1 = y_line1 + position(2);

%  4.2 LINE 2 - Represent the y-axis of the local coordinate frame
%    -> Set the coordinates of the line
x_line2 = [   0     0];
y_line2 = [-3/100  10/100 ];
%   -> Convert these to polar coordinates
[ang, radio] = cart2pol(x_line2,y_line2);
%   -> Convert these back to cartesian coord. but with the new orientation
[x_line2,y_line2] = pol2cart(ang + orientation, radio);
%   -> Set the new location
x_line2 = x_line2 + position(1);
y_line2 = y_line2 + position(2);


%----------------------------------------------------------------------%
% 5. DRAW THE ROBOT
%----------------------------------------------------------------------%
%   5.1. DRAW THE POLYGONS
set(handle_car(1),'xdata',x_car,'ydata',y_car,'facecolor','y');
set(handle_car(2),'xdata',x1_wheels,'ydata',y1_wheels,'facecolor','g');
set(handle_car(3),'xdata',x2_wheels,'ydata',y2_wheels,'facecolor','g');
set(handle_car(4),'xdata',x3_wheels,'ydata',y3_wheels,'facecolor','r');
%   5.2. DRAW THE AXIS
set(handle_car(5),'xdata', x_line1,'ydata',y_line1,'Color','k');
set(handle_car(6),'xdata', x_line2,'ydata',y_line2,'Color','k');


end