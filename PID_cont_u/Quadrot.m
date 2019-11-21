function [dx,ut] = Quadrot(t,x)
g = -9.8;

r = 0.2; % Desired pitch angle %
e = r - x(1); % Pitch error %

% intitialization (need to account for that)
u = 1;

% Enter system dynamics here %
% with no control input
dx = zeros(2,1);
% dx(1) = x(2);
% dx(2) = -g - 2*u;

dx(1) = x(2);
dx(2) =  g + 2*u;

% Need this to get the control signal out later %
if nargout>1
   ut = u;
end
 
 