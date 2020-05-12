function y = gauss_1d(mu, var, x)
% GAUSS_1D	Evaluate a univariate Gaussian distribution.
%
%	Y = GAUSS(MU, VAR, X) evaluates a univariate Gaussian density
%	    of a set of points given by the rows of the matrix X.
%
% OUTPUT
%   Y: The probability density function of the test points
%
% INPUTS
%   mu: Mean of the gaussian
%  var: Variance of the gaussian
%    x: Test points

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. Get number of inputs
N = size(x,1);
%  1.2. Make sure we have a valid variabce
if(var < 1e-20)
    var = 1e-20;
end

%----------------------------------------------------------------------%
% 2. COMPUTE THE GAUSSIAN
%----------------------------------------------------------------------%
%  2.1 COMPUTE THE EXPONENTIAL TERM
y = exp(-0.5*((x - ones(N,1)*mu).^2)./var);
%  2.2. ADD THE NORMALIZATION TERM
y = y./sqrt((2*pi*var));
%  2.3. SET LOWER LIMITS FOR NUMERICAL REASONS
idx = y<1e-10;
y(idx) = 1e-10;


end
