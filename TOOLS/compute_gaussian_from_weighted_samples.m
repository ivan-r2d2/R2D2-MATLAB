function [mu, P, nk] = compute_gaussian_from_weighted_samples(X, PM, PZ, FLAG)
% COMPUTE_GAUSSIAN_FROM_WEIGHTED_SAMPLES Compute gaussian approximation
%
%   [MU, P] = COMPUTE_GAUSSIAN_FROM_SAMPLES2(X) computes the gaussian 
%             statistics of a set of samples.
%
% OUTPUTS
%   mu: Mean of the samples (column)
%    P: Covarianze of the samples
%
% INPUTS
%   X: Set of samples in which each row is a sample.

%----------------------------------------------------------------------%
% 1. ARGUMENTS CHECKING
%----------------------------------------------------------------------%
%  1.1. GET PROPERTIES OF THE SAMPLES
[N, D] = size(X);     % N: Number of samples, D=3
%  1.2. MAKE SURE EACH TEST POSE IS IN ROW
if(D~=3)
    error('COMPUTE_GAUSSIAN_FROM_WEIGHTED_SAMPLES: Each row must be a sample')
end
%  1.3. CHECK DIMENSIONS OF 'PM' Y 'PZ'
if(length(PM)~=N)
    error('COMPUTE_GAUSSIAN_FROM_WEIGHTED_SAMPLES: Bad dimension of PM')
end
if(length(PZ)~=N)
    error('COMPUTE_GAUSSIAN_FROM_WEIGHTED_SAMPLES: Bad dimension of PZ')
end
%  1.4. FLAG
if(nargin<4)
    FLAG = 0;
end


%----------------------------------------------------------------------%
% 2. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  2.1. COMPUTE WEIGHTED SAMPLES
XW = zeros(size(X));
for m=1:N
    XW(m,:) = X(m,:)*PM(m)*PZ(m);
end
%  2.2. VECTOR OF NORMALIZATION TERMS
PM_PZ = PM.*PZ;


%----------------------------------------------------------------------%
% 3. COMPUTE THE MEAN
%----------------------------------------------------------------------%
%  3.1. SUM OF WEIGHTED TERMS IN THE LOCATION
sum_xy = sum(XW(:,[1,2]), 1);        % Along rows
%  3.2. SUM OF WEIGHTED TERMS IN THE ANGLES
%   -> Get indexes of positives angles
pos_angles = X(:,3) >= 0;
%   -> Number of positive and negative angles
num_pos_samples = sum(pos_angles);
num_neg_samples = N - num_pos_samples;
%   -> Normalization term for positive and negative angles
n_pos_angles = sum(PM_PZ(pos_angles));
n_neg_angles = sum(PM_PZ(~pos_angles));
nk = n_pos_angles + n_neg_angles;        % Normalization coeff.
%   -> Compute the mean
if(all(pos_angles) || all(~pos_angles))
    % IF ALL THE ANGLES HAVE THE SAME SIGN(there is no problem)
    sum_angles = sum(XW(:,3),1);
else
    % IF THE ANGLES HAVE DIFFERENT SIGNS(this is a special case)
    %   -> Mean of positive angles
    mean_pos_angles = mean(X(pos_angles,3), 1);
    %   -> Mean of negative angles
    mean_neg_angles = mean(X(~pos_angles,3), 1);
    %   -> Indicators
    suma  = mean_pos_angles + mean_neg_angles;
    resta = mean_pos_angles - mean_neg_angles;
    
    % Compute the mean
    if(resta<pi)
        % IF THE MEAN IS IN THE EXPECTED SIDE
        sum_angles = sum(XW(:,3));
    else
        % IF THE MEAN IS IN THE UNEXPECTED SIDE
        % We make the trick 179, 180, 181, 182,... etc
        X_THETA = X(:,3);
        X_THETA(~pos_angles) = 2*pi + X_THETA(~pos_angles);
        
        % RE-COMPUTE WEIGHTED SAMPLES IN THE ANGLE
        XW_THETA = zeros(N,1);
        for m=1:N
            XW_THETA(m) = X_THETA(m)*PM(m)*PZ(m);
        end
        % SUM
        sum_angles = sum(XW_THETA(:));
    end
    
    % MESSAGES
    if(FLAG)
        fprintf(' mean pos: %2.4f\n', mean_pos_angles*180/pi)
        fprintf(' mean neg: %2.4f\n', mean_neg_angles*180/pi)
        fprintf(' suma: %2.4f\n', suma*180/pi)
        fprintf(' resta: %2.4f\n', resta*180/pi)
        fprintf(' number pos: %d\n', num_pos_samples)
        fprintf(' number neg: %d\n', num_neg_samples)
    end
end
%  3.3. SET TOTAL MEAN
mu = [sum_xy(:);  sum_angles];   % Mean [1x3]
mu = 1/nk*mu;
mu(3) = pi_to_pi(mu(3));


%----------------------------------------------------------------------%
% 4. COMPUTE THE COVARIANZE
%----------------------------------------------------------------------%
%  4.1. MATRIX TO SAVE THE COVARIANCE
P = zeros(D);
%  4.2. OF THE POSITIVE ANGLES
M = X - ones(N,1)*mu';
if(all(pos_angles) || all(~pos_angles))
    for k = 1:N
        mm = M(k,:);
        P = P + mm'*mm*PZ(k)*PM(k);
    end
else
    for k = 1:N
        mm = M(k,:);
        mm(3) = pi_to_pi(mm(3));
        P = P + mm'*mm*PZ(k)*PM(k);
    end
end
P = P/nk;


end