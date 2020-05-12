function e = sigma_ellipse(mu, P, n, nseg)
% SIGMA_ELLIPSE Calculate the ellipse points of a Gaussian function
%
%   Description
%   This function computes the points that correspond to the contour of a 
%   Gaussian distribution.
%
% OUTPUT: 
%     e: Ellipse points [2xNN]
%
% INPUTS: 
%    mu: Mean or center of the gaussian
%     P: Covariance matriz [xx xy; xy yy], 
%     n: Number of sigmas of the contour
%  nseg: Increments

%----------------------------------------------------------------------%
% 1. PARAMETERS CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK NUMBER OF ARGUMENTS
if(nargin < 3)
    error('SIGMA_ELLIPSE: It is necessary 3 arguments')
end
%  1.2. SET NUMBER OF INCREMENTS
if(nargin == 4)
    inc = 2*pi/nseg; 
else
    inc = pi/30;
end

%----------------------------------------------------------------------%
% 2. INITIALIZATION
%----------------------------------------------------------------------%
%  2.1. Compute matrix square root
r = sqrtm(P);       % r*r = P.
%  2.2. Set the angles of the points
phi = 0:inc:2*pi;

%----------------------------------------------------------------------%
% 3. COMPUTE THE POINTS
%----------------------------------------------------------------------%
%  3.1. Compute the contour for a gaussian with mean zero
a = n*r*[cos(phi); sin(phi)];
%  3.2. Add the mean
e(1,:) = a(1,:) + mu(1);        % x
e(2,:) = a(2,:) + mu(2);        % y


end % END