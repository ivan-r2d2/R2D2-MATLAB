function xlocal = point_global_to_local(xglobal, x)
% POINT_GLOBAL_TO_LOCAL  Transform a position in the global reference
%                  	     frame to the local frame of the robot
%
% OUTPUT
%   x_local: Local position of a point(2xN)
%
% INPUTS
%  x_global: Global position of a point(2xN)
%         x: Pose of the robot

%----------------------------------------------------------------------%
% 1. PARAMETERS' CHECKING
%----------------------------------------------------------------------%
%  1.1 CHECK DIMENSIONS OF THE GLOBAL POINTS
[rows, N] = size(xglobal);
if(rows~=2)
    error('POINT_GLOBAL_TO_LOCAL: Number or rows must be 2')
end


%----------------------------------------------------------------------%
% 2. MAIN CODE
%----------------------------------------------------------------------%
%  2.1. TRANSLATE THE GLOBAL COORDINATE FRAME TO THE ORIGIN OF THE
%       LOCAL FRAME
xlocal = xglobal - [x(1); x(2)]*ones(1,N);
%  2.2. ROTATE THE GLOBAL COORDINATES
theta = x(3);
T = [ cos(theta)   sin(theta);
     -sin(theta)   cos(theta)];
xlocal = T*xlocal;


end
