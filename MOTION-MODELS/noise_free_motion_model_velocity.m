function [xp, xc, yc, r, f] = noise_free_motion_model_velocity(x, u, dt)
% NOISE_FREE_MOTION_MODEL Implements the algorithm that describe the exact
%                         motion of a differential-drive mobile robot. See 
%                         Page 127. EQ(5.9)
%
% OUTPUTS
%     xp: Represent the pose at time "t".
%  xc,yc: Coordinates of the center(of the circle of rotation)
%      r: Radious of the circle of rotation
%      f: Flag(1: Rot+trans motion, 0: Only trans. motion)
%
% INPUTS
%    x: Represent the pose "[x;y;theta]" at time "t-1" 
%    u: Vector of controls [v; w];
%   dt: Time interval(seg.)

%----------------------------------------------------------------------%
% 1. GET THE COMPONENTS OF THE PREVIOUS POSE AND THE CONTROLS
%----------------------------------------------------------------------%
%  1.1. GET COMPONENTS OF THE POSE AT 't-1'
xx = x(1);
yy = x(2);
theta = x(3);
if(theta > pi ||theta <= -pi)
    error('NOISE_FREE_MOTION_MODEL: The angle theta is not in the range.')
end
%  1.2. GET THE TRANSLATIONAL AND ROTATIONAL VELOCITIES
v = u(1);
w = u(2);


%----------------------------------------------------------------------%
% 2. COMPUTE THE NEW POSE
%----------------------------------------------------------------------%
% There are two cases:
%   - The robot performs a translational + rotational velocity, so the 
%     robot does a turn around a circle.
%   - The robot performs only a translational velocity, which corresponds
%     to a straight line
%
if(abs(w)>1e-6) % IF THE ROTATIONAL IS NOT ALMOST ZERO
    % 2.1. Compute the coordinates of the center of the circle
    xc = xx - (v/w)*sin(theta);       % EQ(5.7)
    yc = yy + (v/w)*cos(theta);       % EQ(5.8
    % 2.2. Compute the new pose: [xx_p; yy_p; theta_p]. EQ(5.9)
    xx_p = xc + (v/w)*sin(theta + w*dt);
    yy_p = yc - (v/w)*cos(theta + w*dt);
    theta_p = theta + w*dt;
    % 2.3. Set radious of the circle and the flag
    r = abs(v/w);
    f = 1;
else  % IF THE ROTATIONAL VELOCITY IS ALMOST ZERO
    % 2.1. Distance traveled
    d = v*dt;
    % 2.2. Compute the new pose
    xx_p = xx + d*cos(theta);
    yy_p = yy + d*sin(theta);
    theta_p = theta;
    % 2.3. Set the coordinates of the center of the circle
    xc = Inf;
    yc = Inf;
    % 2.4. Set radious of the circle and the flag
    r = Inf;
    f = 0;
end

%----------------------------------------------------------------------%
% 3. SET THE NEW POSE
%----------------------------------------------------------------------%
%  3.1. CHECK THAT THE NEW "theta" IS IN THE RANGE(-pi,pi] 
theta_p = pi_to_pi(theta_p);
%  3.2. SET THE NEW POSE
xp = [xx_p; yy_p; theta_p];

    
end % END
