function display_arc(c, r, theta_o, theta_f, h_fig, color, NN)
% DISPLAY_ARC Plots an arc in the current window
%
% Inputs:
%      c: Coordinates of the center
%      r: Radius of the circle
%theta_o: Initial theta
%theta_f: Final theta
%  h_fig: Handle to the figure where we're going to plot the robot
%  color: Color of the circle
%     NN: Number of points of the circle

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. Check neccesary arguments
if nargin < 5
    error('DISPLAY_ARC: Insufficient number of arguments')
end
%  1.2. Set default color
if nargin < 6
    color = '-b';
end
%  1.3. Set default number of points of the circle
if nargin < 7
    NN = 40;
end
%  1.4 Check the radius
if r <= 0
    error('DISPLAY_ARC The radius is less than or equal to zero.')
end
%  1.5. GET LOCATION
x_r = c(1); 
y_r = c(2);

%----------------------------------------------------------------------%
% 2. PLOT THE ROBOT
%----------------------------------------------------------------------%
% 2.1. Compute the points of the circle
ang = linspace(theta_o, theta_f,NN)';
xx = x_r*ones(NN,1) + r*cos(ang);         % x-components
yy = y_r*ones(NN,1) + r*sin(ang);         % y_components
% 2.2. Plot the circle
%   -> Make the figure identified by "h_fig" the current figure
figure(h_fig)
%   -> Plot
plot(xx,yy, color,'linewidth',2)
hold on


end