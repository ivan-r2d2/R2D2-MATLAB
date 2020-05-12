function p = gauss_ND_robot(X,mu,P)
% GAUSS_ND_robot Evaluate a sample in a Gaussian pdf
%
% OUTPUTS
%   p: Probability p(x|mu,P)
%
% INPUTS:  
%   X: Matriz of poses (Each row is a pose)
%  mu: Mean (in row format)
%   P: Covarianze

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. IF "X" IS A VECTOR MAKE SURE IS A ROW
if(size(X,2) == 1)
    X = X(:)';
end
%  1.2. DATA CONFIGURATION
N = size(X,1);      % Number of points
D = size(X,2);      % Dimension
%  1.3. MEAN CHECKING
if(size(mu,2) == 1)
    mu = mu(:)';        % If "mu" is a vector, make sure is a row
end
if(length(mu) ~= D)
    error(' gauss_ND_robot: Mean and data dimension does not match')
end

%----------------------------------------------------------------------%
% 2. COMPUTE OF THE PROBABILITY
%----------------------------------------------------------------------%
%  2.1. COEFICIENTE DE NORMALIZACION
a = (2*pi)^D;
c = 1/sqrt(a*det(P));
%  2.2. COMPUTE THE TERM "X-u" (row format) 
CC      = X - ones(N,1)*mu;
CC(:,3) = pi_to_pi(CC(:,3));    % To manage the "pi_to_pi"
%  2.3. CALCULO DEL TERMINO (x - mu)'*inv(P)
AA = CC/P;
%  2.4. CALCULO DEL GAUSIANO
p = c*exp(-0.5*sum((AA.*CC),2));
%  2.5. SET LOWER LIMITS FOR NUMERICAL REASONS
idx    = p<1e-10;
p(idx) = 1e-10;


end