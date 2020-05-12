function Xp = sample_motion_model_odometry(X, u, alpha, MOTION_MODEL)
% SAMPLE_MOTION_MODEL_ODOMETRY Implements the odometry motion model
% 
%   SAMPLE_MOTION_MODEL_VELOCITY This is a Matlab-efficient "vectorized"
%   implementation of the algorithm that is described in the table 5.3. 
%   Page 124 of Thruns' book, and also the models described in the
%   papers: .
%
% OUTPUTS
%       Xp: Sample poses at time "t".
%
% INPUTS
%       X: Matrix of poses (each pose is a row)
%       u: Vector of control [drot1; dtrans; drot2] from the encoders
%   alpha: Parameters of the motion noise
%   MODEL: 'STANDARD', 'STANDARD2' 'IMPROVED'

%----------------------------------------------------------------------%
% 1. PARAMETERS CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK THE DIMENSION OF THE VECTOR OF CONTROLS
if(length(u)~=3)
    error('SAMPLE_MOTION_MODEL_ODOMETRY: "u" must be of size 3x1') 
end
%  1.2. CHECK THE DIMENSION OF THE "motion-error" VECTOR OF PARAMETERS
if(length(alpha) ~= 4)
    error('SAMPLE_MOTION_MODEL_ODOMETRY: "alpha" must be of size 4') 
end
%  1.3. IF "X" IS A VECTOR, MAKE SURE X IS IN ROW FORMAT
IS_VECTOR = 0;
if(numel(X)==3)
    IS_VECTOR = 1;
    X = X(:)';
end
%  1.4. IF "X" IS A MATRIX, CHECK IS IN ROW FORMAT
if(size(X,2)~=3)
    error('SAMPLE_MOTION_MODEL_ODOMETRY: "X" must have 3 columns') 
end
%  1.5. MAKE SURE THAT THE ANGLES ARE IN THE CORRECT RANGE (-pi,pi]
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
% 3. COMPUTE ROTATIONAL MOTION IN THE IMPROVED MODEL
%----------------------------------------------------------------------%
if(abs(dtrans)<=0.010 && strcmp(MOTION_MODEL, 'IMPROVED'))
    % GAUSSIAN
    mu = [0;0;drot1+drot2];
    R = diag( [alpha(4)*(abs(drot1)+abs(drot2));...
               alpha(4)*(abs(drot1)+abs(drot2));...
               alpha(1)*(abs(drot1)+abs(drot2))] );
    % TAKE SAMPLES
    samples = mvnrnd(mu,R,NN);
    Xp      = X + samples;
    Xp(:,3) = pi_to_pi(Xp(:,3));
    return
end

%----------------------------------------------------------------------%
% 4. COMPUTE THE TRUE "NOISY" DELTAS
%----------------------------------------------------------------------%
%  4.1. COMPUTE THE VARIANCES
switch(MOTION_MODEL)
    case 'STANDARD'
        var1 = alpha(1)*drot1^2   +  alpha(2)*dtrans^2;
        var2 = alpha(3)*dtrans^2  +  alpha(4)*drot1^2   + alpha(4)*drot2^2;
        var3 = alpha(1)*drot2^2   +  alpha(2)*dtrans^2;
    case {'STANDARD2','IMPROVED'}
        %   -> Compute absolute values
        a_drot1  = abs(drot1);
        a_dtrans = abs(dtrans);
        a_drot2  = abs(drot2);
        %   -> Compute the variances
        var1 = alpha(1)*a_drot1   +  alpha(2)*a_dtrans;
        var2 = alpha(3)*a_dtrans  +  alpha(4)*(a_drot1  +  a_drot2);
        var3 = alpha(1)*a_drot2   +  alpha(2)*a_dtrans;
    otherwise
        error('SAMPLE_MOTION_MODEL_VELOCITY: Unknown motion model')
end
%  4.2. SET THE "true" DELTAS
DROT1_b  = drot1  - sqrt(var1)*randn(NN,1);
DTRANS_b = dtrans - sqrt(var2)*randn(NN,1);
DROT2_b  = drot2  - sqrt(var3)*randn(NN,1);

%----------------------------------------------------------------------%
% 5. GET THE NEW POSES
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
    % B. GET THE CONTROLS
    %------------------------------------------------------------%
    drot1_b  = DROT1_b(m);
    dtrans_b = DTRANS_b(m);
    drot2_b  = DROT2_b(m);
    
    %------------------------------------------------------------%
    % C. COMPUTE THE NEW POSE
    %------------------------------------------------------------%
    xx_p = xx + dtrans_b*cos(theta + drot1_b);
    yy_p = yy + dtrans_b*sin(theta + drot1_b);
    theta_p = theta  +  drot1_b  +  drot2_b;
    
    %------------------------------------------------------------%
    % D. SAVE NEW POSE
    %------------------------------------------------------------%
    xp = [xx_p  yy_p  theta_p];
    Xp(m,:) = xp;
end

%----------------------------------------------------------------------%
% 6. FINAL CONFIGURATION 
%----------------------------------------------------------------------%
%  6.1. MAKE SURE THAT THE NEW POSES' ANGLES ARE IN THE RANGE(-pi,pi] 
Xp(:,3) = pi_to_pi(Xp(:,3));
%  6.2. IF 'X' WAS A VECTOR, make sure the output is a column
if(IS_VECTOR)
    Xp = Xp(:);
end


end