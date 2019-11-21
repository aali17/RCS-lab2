function [dx,ut] = Quadrot(t,x)
% Model of the aircraft system and Control gains %
% kp = 15; % Proportional control gain %
% kd = 3000; % Dirivative control gain (recall this has ? t wrapped in) %
% ki = 0.001; % Integral control gain (recall this has ? t wrapped in) %
g = -9.8;

% kp = 25; % Proportional control gain %
% kd = 1000; % Dirivative control gain (recall this has ? t wrapped in) %
% ki = 0.001; % Integral control gain (recall this has ? t wrapped in) %

% kp = 15; % Proportional control gain %
% kd = 2000; % Dirivative control gain (recall this has ? t wrapped in) %
% ki = 0.001; % Integral control gain (recall this has ? t wrapped in) %

kp = 10; % Proportional control gain %
kd = 1000; % Dirivative control gain (recall this has ? t wrapped in) %
ki = 0.0001; % Integral control gain (recall this has ? t wrapped in) %

r = 50; % Desired pitch angle %
e = r - x(1); % Pitch error %

% PID regulation - need to keep a running total of the error
% as well as the old error for the derivative term %
% ek is the total error accumulated, e_old is the previous error
persistent ek e_old
edot = kd*(e-e_old);
eint = ki*sum(ek);
epro = kp*e;

% First time through e_old will be empty because there's no
% intitialization (need to account for that)
if(isempty(e_old))
   u = 0;
else
   u = edot + eint + epro;
end

% Keep old values for control %
e_old = e;
ek = [ek;e];

% Enter system dynamics here %
% with no control input
dx = zeros(2,1);
dx(1) = x(2);
dx(2) =  g + 2*u;

% Need this to get the control signal out later %
if nargout>1
   ut = u;
end
 
 