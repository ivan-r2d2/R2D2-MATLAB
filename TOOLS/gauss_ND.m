function p = gauss_ND(X,mu,P)
% GAUSS_ND Funcion para evaluar la funcion gausiana en ND
%   
% ENTRADAS:  
%   X: Matriz de puntos de evaluacion. Cada fila es un punto de evaluacion
%  mu: Media(in row format)
%   P: Matriz de covarianza
%
% SALIDA:
%   p  : valor de la funcion gaussiana en cada punto

%----------------------------------------------------------------------%
% 1. CONFIGURACION INICIAL
%----------------------------------------------------------------------%
%  1.1. CONFIGURACION DE LA DATA
N = size(X,1);      % Numero de puntos a evaluar
D = size(X,2);      % Dimension
%  1.2. CONFIGURACION DE LA MEDIA
if(size(mu,2) == 1)
    error(' gauss_ND: El vector mu debe ser una fila');
end
if(length(mu) ~= D)
    error(' gauss_ND: La dimension de la media no coincide con la dimension de la data')
end

%----------------------------------------------------------------------%
% 2. CALCULO DEL GAUSSIANO
%----------------------------------------------------------------------%
%  2.1. COEFICIENTE DE NORMALIZACION
a = (2*pi)^D;
c = 1/sqrt(a*det(P));
%  2.2. COMPUTE "X-u" (row format) 
CC = X - ones(N,1)*mu;
%  2.3. CALCULO DEL TERMINO (x - mu)'*inv(P)
AA = CC/P;
%  2.4. CALCULO DEL GAUSIANO
p = c*exp(-0.5*sum((AA.*CC),2));


end