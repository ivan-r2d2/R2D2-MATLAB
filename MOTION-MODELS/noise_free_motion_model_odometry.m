function Xp = noise_free_motion_model_odometry(X, u)
% NOISE_FREE_MOTION_MODEL_ODOMETRY Implements the "noise-free" motion model
%                                  odometry of a robot.
%
%   XP = NOISE_FREE_MOTION_MODEL_ODOMETRY(U, X) Computes the new pose "xp"
%        of the robot considerend a "noise-free" motion of the robot. That
%        is, we consider as perfect the meassurements of the encoder to
%        predict the next pose of the robot
% 
% OUTPUTS
%   xp: Sample pose at time "t" of the robot.
%
% INPUTS
%   x: Vector that represent the pose "[x,y,theta]" at time "t-1" 
%   u: Vector of controls [drot1; dtrans; drot2] from the encoders

%----------------------------------------------------------------------%
% 1. PARAMETERS CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK THE DIMENSION OF THE VECTOR OF CONTROLS
if(length(u)~=3)
    error('NOISE_FREE_MOTION_MODEL_ODOMETRY: "u" must be of size 3x1') 
end
%  1.2. IF "X" IS A VECTOR, MAKE SURE X IS IN ROW FORMAT
IS_VECTOR = 0;
if(numel(X)==3)
    IS_VECTOR = 1;
    X = X(:)';
end
%  1.3. IF "X" IS A MATRIX, CHECK IS IN ROW FORMAT
if(size(X,2)~=3)
    error('SAMPLE_MOTION_MODEL_ODOMETRY: "X" must have 3 columns') 
end
%  1.4. MAKE SURE THAT THE ANGLES ARE IN THE CORRECT RANGE (-pi,pi]
THETAS = X(:,3);
THETAS_POS = THETAS > pi;
THETAS_NEG = THETAS <= -pi;
if(sum(THETAS_POS) > 0 || sum(THETAS_NEG) > 0)
    error('SAMPLE_MOTION_MODEL_VELOCITY: Some angles are not in the range.')
end


%----------------------------------------------------------------------%
% 2. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  2.1. GET NUMBER OF SAMPLES
NN = size(X,1);
%  2.2. GET THE DELTAS FROM THE ENCODERS
drot1  = u(1);
dtrans = u(2);
drot2  = u(3);


%----------------------------------------------------------------------%
% 3. GET THE NEW POSES
%----------------------------------------------------------------------%
Xp = zeros(size(X));
for m=1:NN
    %------------------------------------------------------------%
    % A. GET A POSE
    %------------------------------------------------------------%
    x = X(m,:);
    xx = x(1);
    yy = x(2);
    theta = x(3);
    
    %------------------------------------------------------------%
    % B. COMPUTE THE NEW POSE
    %------------------------------------------------------------%
    xx_p = xx  +  dtrans*cos(theta + drot1);
    yy_p = yy  +  dtrans*sin(theta + drot1);
    theta_p = theta  +  drot1  +  drot2;
    
    %------------------------------------------------------------%
    % C. SAVE NEW POSE
    %------------------------------------------------------------%
    xp = [xx_p  yy_p  theta_p];
    Xp(m,:) = xp;
end

%----------------------------------------------------------------------%
% 4. FINAL CONFIGURATION 
%----------------------------------------------------------------------%
%  4.1. MAKE SURE THAT THE NEW POSES' ANGLES ARE IN THE RANGE(-pi,pi] 
Xp(:,3) = pi_to_pi(Xp(:,3));
%  4.2. IF 'X' WAS A VECTOR, make sure the output is a column
if(IS_VECTOR)
    Xp = Xp(:);
end

end