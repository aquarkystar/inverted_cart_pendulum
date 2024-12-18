% Define the system parameters
M = 5;          % Mass of the cart (kg)
m = 0.5;        % Mass of the pendulum (kg)
L = 1;          % Length of the pendulum (m)
b = 0.1;        % Damping coefficient (N·s/m)
g = 9.81;       % Gravitational acceleration (m/s^2)

% Define the state-space matrices
A = [0 1 0 0;
     0 -b/M m*g/M 0;
     0 0 0 1;
     0 -m*g/L*(M+m) -b/L*(M+m) 0];

% Corrected B matrix for 2 inputs (force for cart and pendulum torque)
B = [0 1/M;
     0 1/(M*L);
     0 0;
     0 -1/(M*L)];

C = [1 0 0 0;
     0 0 1 0
     0 0 0 0
     0 0 0 0];

D = [0;
     0
     0
     0];

% Uncontrolled system (u = 0)
% Initial conditions [x0, x0_dot, phi0, phi0_dot]
x0 = [0.5; 0; pi+0.1; 0];  % Initial conditions (cart offset, pendulum upright)
t_span = 0:0.01:10;  % Time vector (0 to 10 seconds)

% Simulate the uncontrolled system (no control input u)
A_uncontrolled = A;  % Unchanged A matrix
B_uncontrolled = [0; 0; 0; 0];  % Zero input (no control)
u_uncontrolled = @(t, x) [0];  % Control input is zero

% Solve using ode45 for the uncontrolled system
[~, X_uncontrolled] = ode45(@(t, x) A_uncontrolled * x + B_uncontrolled * u_uncontrolled(t, x), t_span, x0);

% Controlled system using LQR (feedback applied)
% Design LQR controller using pole placement or LQR
poles = [-2, -2.5, -3, -4];  % Desired poles for LQR
K = place(A, B, poles);  % Compute the gain matrix K

u_controlled = @(t, x) -K * x;  % Control input based on feedback

% Solve using ode45 for the controlled system
[~, X_controlled] = ode45(@(t, x) A * x + B * u_controlled(t, x), t_span, x0);

% Plot results for comparison
figure;
subplot(2,1,1);
plot(t_span, X_uncontrolled(:,1), 'r', 'LineWidth', 2);  % Cart position (x)
hold on;
plot(t_span, X_uncontrolled(:,3), 'b', 'LineWidth', 2);  % Pendulum angle (phi)
title('Uncontrolled System');
xlabel('Time (s)');
ylabel('State Variables');
legend('Cart Position (x)', 'Pendulum Angle (\phi)');
grid on;

subplot(2,1,2);
plot(t_span, X_controlled(:,1), 'r', 'LineWidth', 2);  % Cart position (x)
hold on;
plot(t_span, X_controlled(:,3), 'b', 'LineWidth', 2);  % Pendulum angle (phi)
title('Controlled System (LQR)');
xlabel('Time (s)');
ylabel('State Variables');
legend('Cart Position (x)', 'Pendulum Angle (\phi)');
grid on;
