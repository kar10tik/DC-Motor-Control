function dydt= evmodel(t,X)
m = 800; % mass of vehicle (kg)
A = 1.8; % Area (m^2)
rho = 1.25; % Rotor Inertia (kg/m^3)
Cd = 0.3; % 
mu = 0.015; % 
r = 0.25; % vehicle tyre radius (m)
N = 11; % Gear ratio
g = 9.81; % gravitational constant
% a is first derivative of i and b is first derivative of w
L = 0.006008;
R = 0.12;
J = 0.05; B = 2e-4;
Laf = 0.001766; V = 48; q = 0;
a = (1/L)*(V - R*X(1) - Laf*X(1)*X(2));
b = (1/(J + m*(r/N)*(r/N)))*(Laf*X(1)*X(1) - B*X(2) - (r/N)*(mu*m*g*cos(q) + 0.5*rho*A*Cd*((X(2)*r/N)^2) + m*g*sin(q)));
dydt = [a;b];
end