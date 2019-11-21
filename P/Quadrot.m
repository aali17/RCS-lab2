function [dx,ut] = Quadrot(t,x)
g = -9.8;

kp = 1; % Proportional control gain %
r = 50; % Desired pitch angle %
e = r - x(1); % Pitch error %

% ek is the total error accumulated, e_old is the previous error
persistent ek e_old
epro = kp*e;

% First time through e_old will be empty because there's no
% intitialization (need to account for that)
if(isempty(e_old))
   u = 0;
else
   u = epro;
end

% Check and limit actuators here %
% if u < 0
%     u = 0;
% end

% Keep old values for control %
e_old = e;
ek = [ek;e];

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
 
 