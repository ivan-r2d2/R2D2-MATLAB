function [P, drot1_p, dtrans_p, drot2_p, e_rot1, e_trans, e_rot2] =...
    motion_model_odometry(Xp, u, x, alpha, MOTION_MODEL)
% MOTION_MODEL_ODOMETRY Implements the algorithm "motion_model_odometry"
%                       of the improved version
% 
% OUTPUTS
%        p: p(xp | u, x) probability of being at "xp" after executing 
%           control "u" beginning in state "x", assuming that the control 
%           is carried for a fixed duration "delta".
%   drot1_p: The true rotation "drot1" from x to xp
%  dtrans_p: The true translation "dtrans" from x to xp
%   drot2_p: The true rotation "drot2" from x to xp
%    e_rot1: Motion delta error in the first rotation
%   e_trans: Motion delta error in the translation
%    e_rot2: Motion delta error in the second rotation
%
% INPUTS
%      Xp: Matrix that represent the test poses at time "t" (row format)
%       u: Vector of control[drot1; dtrans; drot2] from the encoders
%       x: Vector that represent the pose "[x,y,theta]" at time "t-1" 
%   alpha: Parameters of the motion noise

%----------------------------------------------------------------------%
% 1. PARAMETERS CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK THE DIMENSIONS OF THE VECTOR OF CONTROLS
if(length(u)~=3)
    error('MOTION_MODEL_ODOMETRY: "u" must be of size 3x1') 
end
%  1.2. DEFAULT VALUE OF THE FLAG OF MESSAGES
if(nargin < 6)
    m = 0;
end
%  1.3. IF "Xp" IS A VECTOR, MAKE SURE X IS IN ROW FORMAT
if(numel(Xp)==3)
    Xp = Xp(:)';
end
%  1.4. GET NUMBER OF "Xp" POSES
NN = size(Xp,1);


%----------------------------------------------------------------------%
% 2. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  2.1. GET THE DELTAS FROM THE ENCODERS
drot1  = u(1);
dtrans = u(2);
drot2  = u(3);
%  2.2. COMPUTE VARIANCE OF THE MOTION ERRORS
switch(MOTION_MODEL)
    case 'STANDARD'
        var_rot1  = alpha(1)*drot1^2   +  alpha(2)*dtrans^2;    
        var_trans = alpha(3)*dtrans^2  +  alpha(4)*drot1^2 +...
                    alpha(4)*drot2^2;  
        var_rot2  = alpha(1)*drot2^2   +  alpha(2)*dtrans^2;
    case {'STANDARD2','IMPROVED'}
        a_drot1  = abs(drot1);
        a_dtrans = abs(dtrans);
        a_drot2  = abs(drot2);
        var_rot1  = alpha(1)*a_drot1   + alpha(2)*a_dtrans;
        var_trans = alpha(3)*a_dtrans  + alpha(4)*(a_drot1 + a_drot2);  
        var_rot2  = alpha(1)*a_drot2   + alpha(2)*a_dtrans;
    otherwise
        error('MOTION_MODEL_ODOMETRY: Invalid MOTION MODEL')
end


%----------------------------------------------------------------------%
% 3. COMPUTE ROTATIONAL MOTION IN THE IMPROVED MODEL
%----------------------------------------------------------------------%
if(abs(dtrans)<=0.010 && strcmp(MOTION_MODEL, 'IMPROVED'))
    % SET GAUSSIAN
    mu    = [x(1); x(2); x(3)+drot1+drot2];
    mu(3) = pi_to_pi(mu(3));
    R = diag( [alpha(4)*(abs(drot1)+abs(drot2));...
               alpha(4)*(abs(drot1)+abs(drot2));...
               alpha(1)*(abs(drot1)+abs(drot2))] );
    % COMPUTE DENSITIES
    P = gauss_ND_robot(Xp,mu,R);
    drot1_p=0; dtrans_p=0; drot2_p=0; e_rot1=0; e_trans=0; e_rot2=0;
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
    xp = Xp(n,:);
    
    %------------------------------------------------------------%
    % 4.2. COMPUTE RELATIVE MOTION PARAMETERS
    %------------------------------------------------------------%
    u_real = compute_odometry_controls(x, xp, m);
    drot1_p  = u_real(1);
    dtrans_p = u_real(2);
    drot2_p  = u_real(3);
        
    %------------------------------------------------------------%
    % 4.3. COMPUTE AND CHECK THE MOTION DELTA ERRORS
    %------------------------------------------------------------%
    %  A. ERROR IN "drot1"
    e_rot1 = drot1  - drot1_p;
    e_rot1 = pi_to_pi(e_rot1);
    %  B. ERROR IN "dtrans"
    e_trans = dtrans - dtrans_p;
    %  C. ERROR IN "drot2"
    e_rot2 = drot2  - drot2_p;
    e_rot2 = pi_to_pi(e_rot2);
    
    %------------------------------------------------------------%
    % 4.4. COMPUTE THE PROBABILITY "p : p(xp | u, x)"
    %------------------------------------------------------------%    
    %  A. COMPUTE THE PROBABILITY OF THE ERRORS (mean = '0')
    p_error_rot1  = gauss_1d(0, var_rot1,  e_rot1);
    p_error_trans = gauss_1d(0, var_trans, e_trans);
    p_error_rot2  = gauss_1d(0, var_rot2,  e_rot2);
    %  B. COMPUTE TOTAL PROBABILITY
    pn = p_error_rot1*p_error_trans*p_error_rot2;  
    
    %------------------------------------------------------------%
    % 4.5. SAVE RESULTS
    %------------------------------------------------------------%
    P(n) = pn;
end


end % END