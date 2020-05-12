function display_statistical_parameters(mu,P,FLAG)
% display_statistical_parameters
%
%   DISPLAY_STATISTICAL_PARAMETERS Display in the Command Window the 
%   statistical parameters

%----------------------------------------------------------------------%
% 1. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  1.1. DEFAULT VALUE OF FLAG
if(nargin<3)
    FLAG = 0;
end
%  1.2. GET STD's 
sigmas = sqrt(diag(P));

%----------------------------------------------------------------------%
% 2. DISPLAY MESSAGES
%----------------------------------------------------------------------%
fprintf('     mu: [%2.4f, %2.4f, %2.4f(sex°)]\n',...
    mu(1:2), mu(3)*180/pi)
fprintf('  sigma: [%2.4f, %2.4f, %2.4f(sex°)]\n',...
    sigmas(1:2), sigmas(3)*180/pi)
if(FLAG)
    fprintf('      P: \n')
    disp(P)
end

end