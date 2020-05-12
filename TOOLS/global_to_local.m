function x_local = global_to_local(x_global, frame)
% global_to_local Change from global to local coordinates.
%
% OUTPUTS
%   x_local: Robot in the local frame 
%
% INPUTS
%     frame: Vector that define the frame [tx;ty;theta] in the global frame
%  x_global: Robot in the global frame 


%----------------------------------------------------------------------%
% 1. PARAMETERS OF THE TRANSFORMATION
%----------------------------------------------------------------------%
%  1.1. LOCAL FRAME
tx = frame(1);
ty = frame(2);
theta = frame(3);
%  1.2. ROTATION MATRIX
T = [  cos(theta)   sin(theta);
      -sin(theta)   cos(theta)];

  
%----------------------------------------------------------------------%
% 2. MAIN CODE
%----------------------------------------------------------------------%
%  2.1. TRANSLATION
xx      = x_global(1) - tx;
yy      = x_global(2) - ty;    
x_local = T*[xx;yy];
%  2.2. ROTATION
x_local(3) = x_global(3) - theta;


end
