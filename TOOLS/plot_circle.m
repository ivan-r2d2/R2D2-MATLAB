function plot_circle(c, r, color, NN)
% PLOT_CIRCLE Plot a circle
%
% INPUTS
%      c: Coordinates of the center
%      r: Radius of the circle
%  color: Color of the circle
%     NN: Number of points of the circle

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. CHECK NECCESARY ARGUMENTS
if nargin < 2
    error('plot_circle: Insufficient number of arguments')
end
%  1.2. SET DEFAULT COLOR
if nargin < 3
    color = '-b';
end
%  1.3. SER DEFAULT NUMBER OF POINTS OF THE CIRCLE
if nargin < 4
    NN = 40;
end
%  1.4 CHECK THE RADIOUS
if r <= 0
    error('plot_circle: The radius is less than or equal to zero.')
end
if isinf(r)
    error('plot_circle: The radius is inf.')
end

%----------------------------------------------------------------------%
% 2. GET CENTER OF THE CIRCLE
%----------------------------------------------------------------------%
x_r = c(1); 
y_r = c(2);

%----------------------------------------------------------------------%
% 3. PLOT THE ROBOT
%----------------------------------------------------------------------%
%  3.1. COMPUTE THE POINTS OF THE CIRCLE
ang = linspace(0,2*pi,NN)';
xx = x_r*ones(NN,1) + r*cos(ang);         % x-components
yy = y_r*ones(NN,1) + r*sin(ang);         % y_components
%  3.2. PLOT THE CIRCLE
plot(xx,yy, color)
hold on

end