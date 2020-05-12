function [xp, alpha, R, sign_w, mark] = ...
    compute_pose_from_encoder(x, nrev1, nrev2, D, B, m)
% COMPUTE_POSE_FROM_ENCODER Compute the new pose of the robot using the
%                           lecture of the encoders
%
% OUTPUTS
%       xp: Final "internal odometry" pose predicted by the encoders
%    alpha: Has the following interpretations
%              - If the movement is a pure rotacion around its center, it 
%                gives the angle of rotacion around its center
%              - If the movement is a translation, it gives the distance
%                traveled by the robot
%              - If the movement is a "trans + rot", it gives the angle  
%                of rotacion around the center of the circle
%        R: Has the following interpretations
%              - If the movement is a pure rotacion around its center, it 
%                gives the number of full rotations around its center
%              - If the movement is a translation, it takes "Inf"
%              - If the movement is a "trans + rot", it gives the radious 
%                of the circle on which it rotates
%   sign_w:  Returns the sign of the rotational velocity
%               - "+1": COUNTER-CLOCK WISE
%               - "-1": CLOCKWISE
%               -  "0": w is zero
%     mark:  Has the following values:
%               - 0: If the movement is a pure rotacion around its center
%               - 1: If the movement is a translation
%               - 2: If the movement is a trans + rotation
%
% INPUTS
%       x: Initial "internal odometry" pose of the robot
%   nrev1: Number of revolutions of wheel 1. Right one
%   nrev2: Number of revolutions of wheel 2. Left one
%       D: Diameter of the wheels 
%       B: Distance from each wheel to the center point
%       m: Turn on/off messages
%
%   NOTE: This functions is the same as "compute_pose_from_encoder_hardway" 
%         but with a simpler implementation

%----------------------------------------------------------------------%
% 1. ARGUMENTS' CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK NECCESARY ARGUMENTS
if(nargin < 5)
    error('compute_pose_from_encoder: Insuficient number of arguments')
end
%  1.2. SET DEFAULT VALUE FOR "m"
if(nargin < 6)
    m = 0;
end


%----------------------------------------------------------------------%
% 2. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  2.1. GET THE "internal odometry" POSE AT "t-1"
xx = x(1);
yy = x(2);
theta = x(3);


%----------------------------------------------------------------------%
% 3. MAIN FUNCTION. According to the type of motion(pure rotation, pure
%    translation, or trans+rot) we coompute the outputs
%----------------------------------------------------------------------%
if(abs(nrev1+nrev2) < 5e-3)  % PURE ROTATION
    %  - Note that we allow some degree of error between the number
    %    of revolutions
    % A. SET FLAG AND DISPLAY MESSAGES IF DESIRED
    mark = 0;
    if(m == 1)
        disp('  compute_pose_from_encoder:The movement is a pure rotation around its center')
    end
    % B. COMPUTE THE ARC ROTATED BY THE WHEELS
    L1 = pi*D*nrev1;       % Arc rotated by wheel 1
    L2 = pi*D*nrev2;       % Arc rotated by wheel 2
    arc = (L1-L2)/2;       % Mean 
    % C. ANGLE OF ROTATION AROUND ITS CENTER
    alpha = arc/B;
    % D. COMPUTE THE NEW POSE
    %   -> Set position
    xx_p = xx;
    yy_p = yy;
    %   -> Set the new angle
    theta_p = theta + alpha;
    theta_p = pi_to_pi(theta_p);
    % E. SET NUMBER OF FULL ROTACIONS AROUND ITS CENTER
    R = alpha/(2*pi);
    % F. RETURN THE SIGN OF THE ROTATIONAL VELOCITY(+: COUNTER CLOCKWISE)
    sign_w = sign(alpha);
    
elseif(abs(nrev1-nrev2) < 5e-3)     % PURE TRANSLATION
    %  - Note that we allow some degree of error between the number
    %    of revolutions
    % A. SET FLAG AND DISPLAY MESSAGES IF DESIRED
    mark = 1;
    if(m == 1)
        disp('  compute_pose_from_encoder:The movement is a translation')
    end
    % B. COMPUTE THE ARC ROTATED BY THE WHEELS
    L1 = pi*D*nrev1;
    L2 = pi*D*nrev2;
    % C. DISTANCE TRANSLATED BY THE ROBOT
    d = (L1 + L2)/2;
    % D. COMPUTE THE NEW POSE
    %   -> Set position
    xx_p = xx + d*cos(theta);
    yy_p = yy + d*sin(theta);
    %   -> Set orientation
    theta_p = theta;
    theta_p = pi_to_pi(theta_p);
    % E. SET OTHER VARIABLES
    alpha = d;      % Distance traveled
    R = inf;
    % F. RETURN THE SIGN OF THE ROTATIONAL VELOCITY AS 0
    sign_w = 0;
    
else  % TRANSLATIONAL + ROTATIONAL
    % A. SET FLAG AND DISPLAY MESSAGES IF DESIRED
    mark = 2;
    if(m == 1)
        disp('  compute_pose_from_encoder:The movement is a rotation + translation')
    end
    % B. COMPUTE THE ARC ROTATED BY THE WHEELS
    L1 = pi*D*nrev1;
    L2 = pi*D*nrev2; 
    % C. COMPUTE THE ANGLE OF ROTATION AROUND THE CIRCLE
    alpha = (1/(2*B))*(L1 - L2);
    % D. COMPUTE THE NEW POSE
    if(alpha > 0) % COUNTER-CLOCKWISE
        % Compute the radious of the circle for the center point
        R = L2/alpha + B;
        % Compute the new pose
        %   -> Set position
        phi = alpha + theta;
        xx_p = xx + ( R*sin(phi) - R*sin(theta));
        yy_p = yy + (-R*cos(phi) + R*cos(theta));    
        %   -> Set the new orientation
        theta_p = phi;
        theta_p = pi_to_pi(theta_p);
    else %CLOCKWISE
        % Compute the radious of the circle for the center point
        R = L1/(abs(alpha)) + B;
        % Compute the new pose
        %   -> Set position
        phi = alpha + theta;
        xx_p = xx + (-R*sin(phi) + R*sin(theta));
        yy_p = yy + ( R*cos(phi) - R*cos(theta));
        %   -> Set the new orientation
        theta_p = phi;
        theta_p = pi_to_pi(theta_p);
    end
    % E. RETURN THE SIGN OF THE ROTATIONAL VELOCITY(+: COUNTER CLOCKWISE)
    sign_w = sign(alpha);
end
%  SET THE NEW POSE
xp = [xx_p; yy_p; theta_p];


end % END