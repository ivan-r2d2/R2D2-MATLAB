function [P, v_true, w_true, gamma, mu, xc, yc, r, delta_theta] = ...
    motion_model_velocity(Xp, u, x, dt, alpha, MOTION_MODEL, m)
% MOTION_MODEL_VELOCITY Implements the algorithm "motion_model_velocity"
%                       that is described in the table 5.1. Page 123.
% 
% OUTPUTS
%        P: p(xp | u, x) probability of being at "xp" after executing 
%           control "u" beginning in state "x", assuming that the control 
%           is carried for a fixed duration "dt".
%   v_true: The true trans. velocity for the robot to reach "xp"
%   w_true: The true rot. velocity for the robot to reach "xp"
%    gamma: Additional rotational velocity to achieve the final heading
%       mu: Distance from the center circle to the half-way point
%           between "x" and "xp"
%   xc, yc: Coordinates of the center of the circle
%        r: Radius of the circle 
%  delta_theta: Change in heading direction
%
% INPUTS
%      Xp: Matrix that represent the test poses at time "t" (row format)
%       u: Vector of control = [v; w];
%       x: Vector that represent the pose "[x,y,theta]" at time "t-1" 
%      dt: Time interval(seg.)
%   alpha: Robot-specific motion error parameters
%  MOTION: 'STANDARD', 'IMPROVED'
%       m: Turn on/off some messages

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. SET DEFAULT VALUE FOR "m"
if(nargin < 7)
    m = 0;
end
%  1.2. IF "Xp" IS A VECTOR, MAKE SURE X IS IN ROW FORMAT
if(numel(Xp)==3)
    Xp = Xp(:)';
end
%  1.3. GET NUMBER OF "Xp" POSES
NN = size(Xp,1);

%----------------------------------------------------------------------%
% 2. GET "xp" AND CONTROLS
%----------------------------------------------------------------------%
%  2.1. GET THE COMPONENTS OF THE POSE "x(t-1)"
xx = x(1);
yy = x(2);
theta = x(3);
%  2.2. GET THE VELOCITIES
v = u(1);
w = u(2);
%  2.3. CHECK MODEL
switch(MOTION_MODEL)
    case 'STANDARD'
        var_error_v     = alpha(1)*v^2 + alpha(2)*w^2;
        var_error_w     = alpha(3)*v^2 + alpha(4)*w^2;
        var_error_gamma = alpha(5)*v^2 + alpha(6)*w^2;
    case {'STANDARD2','IMPROVED'}
        var_error_v     = 1/dt*(alpha(1)*v^2 + alpha(2)*w^2);
        var_error_w     = 1/dt*(alpha(3)*v^2 + alpha(4)*w^2);
        var_error_gamma = 1/dt*(alpha(5)*v^2 + alpha(6)*w^2);
    otherwise
        error('MOTION_MODEL_VELOCITY: Invalid MOTION MODEL')
end

%----------------------------------------------------------------------%
% 3. COMPUTE ROTATIONAL MOTION IN THE IMPROVED MODEL
%----------------------------------------------------------------------%
if(abs(u(1))<=1e-4 && strcmp(MOTION_MODEL, 'IMPROVED'))
    % SET GAUSSIAN
    muG    = x + [0;0;w*dt];
    muG(3) = pi_to_pi(muG(3));
    R = diag( [ alpha(2)*(w^2)*dt/1.1412;...
                alpha(2)*(w^2)*dt/1.4142;...
               (alpha(4) + alpha(6))*(w^2)*dt] );
    % COMPUTE DENSITIES
    P = gauss_ND_robot(Xp,muG,R);
    % SET OTHER OUTPUTS
    xc = -100; yc=-100; r=-100;
    v_true=-100; w_true=-100; gamma=-100; 
    delta_theta=-100;
    return
end

%----------------------------------------------------------------------%
% 4. COMPUTE THE PROBABILITIES p(xp | u, x)
%----------------------------------------------------------------------%
P = zeros(NN,1);
for n=1:NN
    %------------------------------------------------------------%
    % 4.1. GET A "xp" POSE
    %------------------------------------------------------------%
    %  -> Get a pose
    xp = Xp(n,:);
    %  -> Get its components
    xx_p    = xp(1);
    yy_p    = xp(2);
    theta_p = xp(3);
    
    %--------------------------------------------------------------%
    % 4.2. MAIN CODE
    %--------------------------------------------------------------%
    %  A. COMPUTE "mu": distance from the center circle to the
    %      half-way point between "x" and "xp"
    num = (xx - xx_p)*cos(theta) + (yy - yy_p)*sin(theta);
    den = (yy - yy_p)*cos(theta) - (xx - xx_p)*sin(theta);
    mu = 0.5*num./den;
    %  B. ACCORDING TO THE KIND OF MOTION WE COMPUTE:
    %        -> the true translacional velocity
    %        -> the true rotational velocity
    %        -> the corrected delta_theta
    if(abs(xx - xx_p)== 0 && abs(yy - yy_p)== 0)
        % a. THE POSITION AT "t-1" AND "t" ARE ALMOST THE SAME
        %   - Show message if desired
        if(m == 1)
            disp('  The real motion is to be static or pure ideal rotational')
        end
        %   - Set true velocities
        v_true      = 0;
        delta_theta = pi_to_pi(theta_p - theta);
        w_true      = delta_theta/dt;
        %   - Set other variables
        xc = inf;   yc = inf;
        r = inf;    mu = inf;
            
    elseif(abs(mu) > 1e5)
        % b. THE RADIUS OF THE CIRCLE IS VERY BIG, SO THE MOTION IS
        %    CONSIDERED TO BE TRANSLATIONAL
        %   - Show message if desired
        if(m == 1)
            disp('  The real motion for this test point is translational')
        end
        %   - Distance traveled
        d = sqrt((yy_p - yy)^2 + (xx_p - xx)^2);
        %   - Get the angle of the movement 
        theta_t = atan2(yy_p - yy, xx_p - xx);
        %   - Compute the true translational velocities.
        if (sign(theta_t) == sign(theta))
            v_true = d/dt;       % Forward motion
        else
            v_true = -d/dt;      % Backward motion
        end
        %   - Set the rotational velocities and change in angle
        w_true = 0;
        delta_theta = 0;
        %   - Set other variables
        xc = inf;   yc = inf;
        r = inf;

    else % TRANSLATIONAL + ROTATIONAL
        %   - Show message if desired
        if(m==1)
            disp('  The real motion for this test point is trans + rot')
        end
        % COORDINATES OF THE CENTER
        %     NOTE: In some cases "mu" is negative. This happens when the test
        %           point is more than 180�
        xc = 0.5*(xx + xx_p) + mu.*(yy - yy_p);
        yc = 0.5*(yy + yy_p) + mu.*(xx_p - xx);    
        % RADIUS OF THE CIRCLE
        r = sqrt((xx - xc).^2 + (yy - yc).^2);    
        % CHANGE OF HEADING DIRECTION
        theta_f = atan2(yy_p - yc, xx_p - xc);
        theta_f = pi_to_pi(theta_f);
        theta_o = atan2(yy - yc, xx - xc);
        theta_o = pi_to_pi(theta_o);
        delta_theta = pi_to_pi(theta_f - theta_o);
        % TRUE VELOCITIES
        w_true = delta_theta/dt;
        v_true = (abs(delta_theta)/dt)*r;
        
        % CORRECT VELOCITIES FOR BACKWARD MOTION
        vector = [xx_p-xx; yy_p-yy];
        M = [cos(theta)  sin(theta); -sin(theta)  cos(theta)];
        vector_p = M*vector;
        if(vector_p(1) < 0 )
            v_true = -v_true;
            if(m==1)
                disp('  ...doing correction')
            end
        end
    end
    %  C. ADDITIONAL ROTATIONAL VELOCITY
    gamma = pi_to_pi(theta_p-theta)/dt - w_true;
    
    %----------------------------------------------------------------%
    % 4.3 COMPUTE THE PROBABILITY "p : p(xp | u, x)"
    %----------------------------------------------------------------%
    %  A. COMPUTE VELOCITY ERRORS
    error_v  = v - v_true;
    error_w  = w - w_true;
    error_gamma = gamma;
    %  B. COMPUTE PROBABILITY ERRORS (mean = '0')
    p_error_v     = gauss_1d(0, var_error_v, error_v);
    p_error_w     = gauss_1d(0, var_error_w, error_w);
    p_error_gamma = gauss_1d(0, var_error_gamma, error_gamma);
    %  C. RESULT
    pn = p_error_v.*p_error_w.*p_error_gamma;
    
    %----------------------------------------------------------------%
    % 4.4. SAVE RESULTS
    %----------------------------------------------------------------%
    P(n) = pn;
end


end % END
