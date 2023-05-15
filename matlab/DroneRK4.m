function new_x = DroneRK4(x, u, dt)
k1 = DroneEuler(x, u);
k2 = DroneEuler(x + k1*dt/2, u);
k3 = DroneEuler(x + k2*dt/2, u);
k4 = DroneEuler(x + k3*dt, u);
new_x = x + dt*(k1 + 2*k2 + 2*k3 + k4) / 6;
end