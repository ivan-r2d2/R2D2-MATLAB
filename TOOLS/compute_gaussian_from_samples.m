function [mu, P] = compute_gaussian_from_samples(X, FLAG)
% COMPUTE_GAUSSIAN_FROM_SAMPLES Compute gaussian approximation
%
%   COMPUTE_GAUSSIAN_FROM_SAMPLES(X) computes the gaussian 
%   statistics of a set of robot pose samples.
%
% OUTPUTS
%   mu: Mean of the samples [3x1]
%    P: Covarianze of the samples [3x3]
%
% INPUTS
%      X: Set of samples in which each row is a sample.
%   FLAG: Flag to show debug messages

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. SET DEFAULT VALUE FOR 'FLAG'
if(nargin<2)
    FLAG = 0;
end
%  1.2. CHECK FORMAT OF 'X'
if(size(X,2)~=3)
    error('COMPUTE_GAUSSIAN_FROM_SAMPLES: Each row must be a sample')
end
%  1.3. GET NUMBER OF SAMPLES
N = size(X,1);
%  1.4. Dimensions of the samples
D = size(X,2);      % D=3


%----------------------------------------------------------------------%
% 2. COMPUTE THE MEAN
%----------------------------------------------------------------------%
%  2.1. MEAN IN THE LOCATION
mu_xy = mean(X(:,[1,2]), 1);            % Along rows
%  2.2. MEAN IN THE ANGLES
%   -> Get indexes of positives angles
angles = X(:,3) >= 0;
num_pos_samples = sum(angles);          % Number of positive samples
num_neg_samples = N - num_pos_samples;  % Number of negative samples
%   -> Compute the mean
if(all(angles) || all(~angles))
    % If all the angles have the same sign there is no problem
    mean_angles = mean(X(:,3));
else
    % If not, this is a special case
    %   -> Mean of positive angles
    mean_pos_angles = mean(X(angles,3), 1);
    %   -> Mean of negative angles
    mean_neg_angles = mean(X(~angles,3), 1);
    %   -> Indicators
    suma  = mean_pos_angles + mean_neg_angles;
    resta = mean_pos_angles - mean_neg_angles;
    
    % Compute the mean
    if(resta<pi)
        % IF THE MEAN IS IN THE EXPECTED SIDE
        mean_angles = mean(X(:,3));
    else
        % IF THE MEAN IS IN THE UNEXPECTED SIDE
        % We make the trick 179, 180, 181, 182,... etc
        XX = X;
        XX(~angles,3) = 2*pi + XX(~angles,3);
        mean_angles = mean(XX(:,3));
        if(mean_angles>pi)
            mean_angles = mean_angles - 2*pi;
        end
    end
    
    % DEBUG MESSAGES
    if(FLAG)
        fprintf(' mean pos: %2.4f\n', mean_pos_angles*180/pi)
        fprintf(' mean neg: %2.4f\n', mean_neg_angles*180/pi)
        fprintf(' suma: %2.4f\n', suma*180/pi)
        fprintf(' resta: %2.4f\n', resta*180/pi)
        fprintf(' number pos: %d\n', num_pos_samples)
        fprintf(' number neg: %d\n', num_neg_samples)
    end
end
%  2.3. SET TOTAL MEAN
mean_angles = pi_to_pi(mean_angles);    % Mean back to (-pi, pi]
mu = [mu_xy(:)'  mean_angles];          % Mean [1x3]


%----------------------------------------------------------------------%
% 3. COMPUTE THE COVARIANCE
%----------------------------------------------------------------------%
%  3.1. MATRIX TO SAVE THE COVARIANCE
P = zeros(D);
%  3.2. COMPUTE MATRIX
M = X - ones(N,1)*mu;
M(:,3) = pi_to_pi(M(:,3));
for n = 1:N
    P = P + M(n,:)'*M(n,:);
end
%  3.3. TO MAKE SURE IS VALID
P = (P + P')/2;
P = P + 0.00001*eye(size(P));   % CHECK THIS FOR 
                                % Warning: Matrix is singular and may 
                                % not have a square root. 
%  3.4. RETURN UNBIASED COVARIANCE
P = (1/(N-1))*P;
mu = mu(:);


end
