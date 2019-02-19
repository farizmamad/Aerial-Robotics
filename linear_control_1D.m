% The state x and control input u
% altitude-z = x(1)
% velocity-z = x(2) = x(1)dot
% u = x(2)dot (this is control input)

% in terms of quadrotor flight in z-direction, 
% u(t) = xdesdotdot(t) + k_d(xdesdot(t) - xdot(t)) + k_p(xdes(t) - x(t))
% Here, it is assumed that u(t) can reach any value, it means that it has no limitation.

% xdes = constant. Thus xdesdot = 0 and xdesdotdot = 0
xdes = 1;
xdesdot = 0
xdesdotdot = 0

% The system of 1st order ODE become:
% dxdt = [x(1)dot, x(2)] = [x(2), u] = [x(2); xdesdotdot + k_d(xdesdot - x(2)) + k_p(xdes - x(1))]

kd = 2; % chosen derivative gain
kp = 3; % chosen proportional gain
dxdt = @(t,x)[x(2); xdesdotdot + (kd*(xdesdot - x(2))) + (kp*(xdes - x(1)))]

% define timespace/horizon
tLim = [0:0.1:10]

% define initial condition
x0 = [0, 0]

[tSol, xSol] = ode45(dxdt,tLim,x0)

plot(tSol, xSol(:,1), 'g')
hold on
plot(tSol, ones(length(xSol(:,1)),1), 'r--')
