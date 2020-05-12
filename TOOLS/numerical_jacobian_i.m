function J = numerical_jacobian_i(model, idx, offset, varargin)
% NUMERICAL_JACOBIAN_I Compute the numerical Jacobian of a function
%
% OUTPUT
%    J: Jacobian dy/dxi computed about the selected arguments. In our 
%       case can be the state "x" or the vector of controls "u"
%
% INPUTS:
%     model: Handle to the function "y = model(x, u, dt)"
%       idx: Index of the model argument in which to compute dy/dxi
%    offset: (optional) finite difference for approximating hyperplane
%  varargin: Adittional arguments required for the evaluation of
%            the function model. In our case the specific values of
%            "(x, u, dt)". (The point of derivation)
%
%
% REMARKS:
%   1. The function handler can be passed as @model or 'model'. 
%   2. The offset argument is optional (defaults to 1e-9).
%   3. Jacobian matrix is computed via central differencing. This is a
%      quick and nasty way to compute derivatives and one should consider
%      automatic differentiation if more accurate results are required.

%----------------------------------------------------------------------%
% 1. PARAMETERS CHECKING
%----------------------------------------------------------------------%
%  1.1. CHECK NUMBER OF ARGUMENTS
if nargin < 4
    error('NUMERICAL_JACOBIAN_I: It is required 4 arguments')
end
%  1.2. CHECK OFFSET
if (isempty(offset))
    offset = 1e-9; 
end


%----------------------------------------------------------------------%
% 2. INITIAL CONFIGURATION
%----------------------------------------------------------------------%
%  2.1. GET VECTOR "x" OF DERIVATION(can be the pose,the controls , or other)
x = varargin{idx};
%  2.2. EVALUATE THE MODEL WITH THE CURRENT GIVEN PARAMATERS (in our model:
%       the state "varargin{1}", the controls "varargin{2}" and the
%       delta of time dt "varargin'{3}")
%       -> NOTE: Make sure that the order of varargin{:} correspond to the
%                order of the arguments of the function "model"
y = feval(model, varargin{:});
%  2.3. GET THE DIMENSIONS ON THE OUTPUTS AND THE INPUTS. (In the case of
%       the mobile robot: "y=[3x1], x=[3x1, for derivation in the state] 
%       or x=[2x1, for derivation in the control]"
leny = length(y);       % Number of outputs of the model
lenx = length(x);       % Dimension of the vector of derivation


%----------------------------------------------------------------------%
% 3. COMPUTE THE JACOBIANS
%----------------------------------------------------------------------%
%  3.1. Pre-allocate array for speed
J = zeros(leny, lenx);
%  3.2. Compute the derivation for each dimension
for i=1:lenx
    % a. Compute small desviations in each component of the vector x=[xx,yy,theta]
    xu = x(i) + offset;
    xl = x(i) - offset;
    % b. Compute the function model on the desviations 
    varargin{idx}(i) = xu;
    yu = feval(model, varargin{:}); 
    varargin{idx}(i) = xl;
    yl = feval(model, varargin{:});    
    % c. Put again the initial value
    varargin{idx}(i) = x(i);
    % d. Compute the Jacobian
    %   - Set numerator
    dy = yu - yl;
    %   - Jacobian
    J(:,i) = dy/(xu - xl);  
    % Numerically better to divide by xu - xl rather than 2*offset. 
end


end % END