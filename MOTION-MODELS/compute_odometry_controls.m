function u = compute_odometry_controls(x_bar, xp_bar, m)
% COMPUTE_ODOMETRY_CONTROLS Compute the delta's of the encoder
%
%   U = COMPUTE_ODOMETRY_CONTROLS(X_BAR, XP_BAR) Computes the odometry
%       controls "deltas" from the encoders. See Page 138
% 
% OUTPUTS
%       u: Vector of odometry controls [3x1]
%
% INPUTS
%   x_bar: Pose of the robot according to the encoder at time "t-1" 
%  xp_bar: Pose of the robot according to the encoder at time "t"

%----------------------------------------------------------------------%
% 1. PARAMETERS CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK THE DIMENSIONS OF "x_bar"
if(length(x_bar)~=3)
    error('COMPUTE_ODOMETRY_CONTROLS: "x_bar" must be of size 3x1') 
end
%  1.2. CHECK THE DIMENSIONS OF "xp_bar"
if(length(xp_bar)~=3)
    error('COMPUTE_ODOMETRY_CONTROLS: "xp_bar" must be of size 3x1') 
end
%  1.3. SET DEFAULT VALUE FOR "m"
if(nargin<3)
    m=0;
end

%----------------------------------------------------------------------%
% 2. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  2.1. GET THE POSE "x_bar" FROM THE ENCODER AT TIME "t-1" 
xx_b    = x_bar(1);
yy_b    = x_bar(2);
theta_b = x_bar(3);
%  2.2. GET THE POSE "xp_bar" FROM THE ENCODER AT TIME "t" 
xxp_b    = xp_bar(1);
yyp_b    = xp_bar(2);
thetap_b = xp_bar(3);

%----------------------------------------------------------------------%
% 3. COMPUTE THE "DELTAS" THAT CORRESPOND TO THE MEASSUREMENT OF THE
%    ENCODERS
%----------------------------------------------------------------------%
%  3.1. COMPUTE THE DELTA OF THE TRANSLATION
dtrans = sqrt((xx_b - xxp_b)^2  + (yy_b - yyp_b)^2);
%  3.2. COMPUTE DELTAS OF ROTATION
if(dtrans>0.0)   % if(dtrans~=0)
    drot1 = atan2(yyp_b - yy_b,xxp_b - xx_b) - theta_b;
    drot2 = thetap_b - theta_b - drot1;
else % Pure rotation
    % We put these to correct some 'curious' cases  =
    % See Demo_02 (step:6,7) and Demo_11(step:6,7)
    Dth   = thetap_b - theta_b;
    Dth   = pi_to_pi(Dth);
    drot1 = 0.5*Dth;
    drot2 = 0.5*Dth;
end
%drot1 = atan2(yyp_b - yy_b,xxp_b - xx_b) - theta_b;
%drot2 = thetap_b - theta_b - drot1;
%  3.3. CHECK DELTAS
drot1 = pi_to_pi(drot1);
drot2 = pi_to_pi(drot2);
%  3.4. MESSAGES
if(m==1)
    disp(' COMPUTE_ODOMETRY_CONTROLS:Original values')
    fprintf('     drot1: %2.4f, d_trans: %2.4f, drot2: %2.4f\n',...
        drot1,dtrans,drot2)
end

%----------------------------------------------------------------------%
% 4. DO CORRECTIONS FOR BACKWARDS MOTION 
%----------------------------------------------------------------------%
if(abs(drot1)>pi/2 && dtrans~=0)
    % Correct 'drot1', 'dtrans<0'
    drot1  = drot1 - sign(drot1)*pi;
    dtrans = -dtrans;
    % Recompute 'drot2'
    drot2 = thetap_b - theta_b - drot1;
    drot2 = pi_to_pi(drot2);
end

%----------------------------------------------------------------------%
% 5. OUTPUT
%----------------------------------------------------------------------%
u = [drot1; dtrans; drot2];


end
