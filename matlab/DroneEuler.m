function dx = DroneEuler(x, u)

% Parameters.
alpha = 1;
e3 = [0; 0; 1];
g = 9.81; % gravitational acceleration
m = 1; % mass
I = diag([0.03, 0.03, 0.06]); % moment of inertia matrix

% p = x(1: 3); % position
q = x(4: 7); % quaternion
v = x(8: 10); % linear velocity
f = x(11);

df = u(1); % thrust
w = u(2: 4); % angular velocity

dp = v;
dq = QuatProd(q, w) / 2 - alpha * (q'*q - 1) * q;
tmp = QuatProd(q, e3);
tmp = QuatProd(tmp, [q(1); -q(2:4)]);
dv = -g *e3 + f / m * tmp(2:4);

dx = [dp; dq; dv; df];
end