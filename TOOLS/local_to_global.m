function x_global = local_to_global(x_local, frame)
% local_to_global Compute the global configuration
%
% OUTPUTS
%   x_global: Pose in the global frame
%
% INPUTS
%    frame: Vector that define the frame [tx;ty;theta] in the global frame
%  x_local: Pose in the local frame 

%----------------------------------------------------------------------%
% 1. PARAMETERS OF THE TRANSFORMATION
%----------------------------------------------------------------------%
%  1.1. LOCAL FRAME
tx = frame(1);
ty = frame(2);
theta = frame(3);
%  1.2. ROTATION MATRIX
T = [ cos(theta)   -sin(theta);
      sin(theta)   cos(theta)];
    

%----------------------------------------------------------------------%
% 2. MAIN CODE
%----------------------------------------------------------------------%
%  2.1. COMPUTE POSITION
x_global    = T*x_local(1:2);
x_global    = x_global + [tx;ty];
%  2.2. COMPUTE ORIENTATION
x_global(3) = pi_to_pi(theta + x_local(3));


end
