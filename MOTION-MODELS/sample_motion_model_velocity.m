function Xp = sample_motion_model_velocity(X, u, dt, alpha, MOTION_MODEL)
% SAMPLE_MOTION_MODEL_VELOCITY Sample from the velocity model
% 
%   SAMPLE_MOTION_MODEL_VELOCITY This is a Matlab-efficient "vectorized"
%   implementation of the algorithms that is described in the table 5.3. 
%   Page 124 of Thruns' book and also see in my paper.
%
% OUTPUT
%     Xp: Matrix of new poses
%
% INPUT
%      X: Matrix of poses (row format)
%      u: Vector of control = [v; w];
%     dt: Time interval(seg.)
%  alpha: Parameters of the motion noise
%  MODEL: 'STANDARD', 'IMPROVED'

%----------------------------------------------------------------------%
% 1. PARAMETERS CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK THE DIMENSIONS OF THE VECTOR OF CONTROLS
if(length(u)~=2)
    error('SAMPLE_MOTION_MODEL_VELOCITY: "u" must be of size 2x1') 
end
%  1.2. CHECK THE DIMENSIONS OF THE VECTOR OF NOISE
if(length(alpha) ~= 6)
    error('SAMPLE_MOTION_MODEL_VELOCITY: Vector of errors must be of size 6')
end
%  1.3. IF "X" IS A VECTOR, MAKE SURE X IS IN ROW FORMAT
IS_VECTOR = 0;
if(numel(X)==3)
    IS_VECTOR = 1;
    X = X(:)';
end
%  1.4. IF "X" IS A MATRIX, CHECK IT IS IN ROW FORMAT
if(size(X,2)~=3)
    error('SAMPLE_MOTION_MODEL_VELOCITY: X must have 3 columns')
end
%  1.5. MAKE SURE THAT THE ANGLES ARE IN THE CORRECT RANGE (-pi,pi]
THETAS = X(:,3);
THETAS_POS = THETAS > pi;
THETAS_NEG = THETAS <= -pi;
if(sum(THETAS_POS) > 0 || sum(THETAS_NEG) > 0)
    error('SAMPLE_MOTION_MODEL_VELOCITY: Some angles are not in the range.')
end


%----------------------------------------------------------------------%
% 2. COMPUTE ROTATIONAL MOTION IN THE IMPROVED MODEL
%----------------------------------------------------------------------%
if(abs(u(1))<=1e-4 && strcmp(MOTION_MODEL, 'IMPROVED'))
    % GET 'w' AND NUMBER OF SAMPLES
    w = u(2);
    NN = size(X,1);
    % COMPUTE GAUSSIAN
    mu = [0;0;w*dt];
    R = diag( [ alpha(2)*(w^2)*dt/1.1412;...
                alpha(2)*(w^2)*dt/1.4142;...
               (alpha(4) + alpha(6))*(w^2)*dt] );
    % TAKE SAMPLES         
    samples = mvnrnd(mu,R,NN);
    Xp      = X + samples;
    Xp(:,3) = pi_to_pi(Xp(:,3));
    return
end


%----------------------------------------------------------------------%
% 3. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  3.1. GET NUMBER OF SAMPLES
NN = size(X,1);
%  3.2. SET TRUE VELOCITIES - The comanded velocities perturbed by noise
%   -> Set variances
switch(MOTION_MODEL)
    case 'STANDARD'
        var_v     = alpha(1)*u(1)^2 + alpha(2)*u(2)^2;
        var_w     = alpha(3)*u(1)^2 + alpha(4)*u(2)^2;
        var_gamma = alpha(5)*u(1)^2 + alpha(6)*u(2)^2;
    case {'STANDARD2','IMPROVED'}
        var_v     = 1/dt*(alpha(1)*u(1)^2 + alpha(2)*u(2)^2);
        var_w     = 1/dt*(alpha(3)*u(1)^2 + alpha(4)*u(2)^2);
        var_gamma = 1/dt*(alpha(5)*u(1)^2 + alpha(6)*u(2)^2);
    otherwise
        error('SAMPLE_MOTION_MODEL_VELOCITY: Unknown motion model')
end
% -> Set velocities
V   = u(1) + sqrt(var_v)*randn(NN,1);
W   = u(2) + sqrt(var_w)*randn(NN,1);
GAMMA = sqrt(var_gamma)*randn(NN,1);


%----------------------------------------------------------------------%
% 4. GET THE NEW POSES
%----------------------------------------------------------------------%
Xp = zeros(size(X));
for m=1:NN
    %------------------------------------------------------------%
    % 4.1. GET A POSE
    %------------------------------------------------------------%
    x = X(m,:);
    theta = x(3);
    
    %------------------------------------------------------------%
    % 4.2. GET THE CONTROLS
    %------------------------------------------------------------%
    v = V(m);
    w = W(m);
    gamma = GAMMA(m);
    
    %------------------------------------------------------------%
    % 4.3. COMPUTE NEW POSE
    %------------------------------------------------------------%
    if(abs(w)>1e-6)
        % IF THE "true" ROTATIONAL VELOCITY IS NOT ALMOST ZERO
        xx_p = x(1) - (v/w)*sin(theta) + (v/w)*sin(theta + w*dt);
        yy_p = x(2) + (v/w)*cos(theta) - (v/w)*cos(theta + w*dt);
        theta_p = theta + w*dt + gamma*dt;
    else
        % IF THE "true" ROTATIONAL VELOCITY IS ALMOST ZERO
        %  a. Distance traveled
        d = v*dt;
        %  b. New pose
        xx_p = x(1) + d*cos(theta);
        yy_p = x(2) + d*sin(theta);
        theta_p = theta + gamma*dt;
    end
    
    %------------------------------------------------------------%
    % 4.4. SAVE NEW POSE
    %------------------------------------------------------------%
    xp = [xx_p  yy_p theta_p];
    Xp(m,:) = xp;
end


%----------------------------------------------------------------------%
% 5. FINAL CONFIGURATION 
%----------------------------------------------------------------------%
%  5.1. MAKE SURE THAT THE NEW POSES' ANGLES ARE IN THE RANGE(-pi,pi] 
Xp(:,3) = pi_to_pi(Xp(:,3));
%  5.2. IF 'X' WAS A VECTOR, MAKE SURE "Xp" IS A COLUMN
if(IS_VECTOR)
    Xp = Xp(:);
end


end % END