% Define system parameters
m_cart = 5;       % Mass of the cart (kg)
m_pendulum = 0.5; % Mass of the pendulum (kg)
L = 1;            % Length of the pendulum (m)
b = 0.1;          % Damping coefficient (N.s/m)
g = 9.81;         % Gravitational acceleration (m/s^2)

% Define the system matrices A, B, C, and D
A = [0      1            0              0; 
     0  -b/m_cart   m_pendulum*g/m_cart     0; 
     0      0            0              1; 
     0  -b/(m_cart*L)  -(m_cart+m_pendulum)*g/(m_cart*L)  0];

B = [0; 
     1/m_cart; 
     0; 
     1/(m_cart*L)];

C = [1 0 0 0;   % Measuring the position of the cart
     0 0 1 0
     0 0 0 0
     0 0 0 0];   % Measuring the angle of the pendulum

D = [0;  % Non-zero direct feedthrough from input to cart position output
     0;
     0;
     0];  % Non-zero direct feedthrough from input to pendulum angle output

% Check matrix dimensions to avoid inconsistency
disp('A matrix dimensions:');
disp(size(A));

disp('B matrix dimensions:');
disp(size(B));

disp('C matrix dimensions:');
disp(size(C));

disp('D matrix dimensions:');
disp(size(D));

% Calculate the controllability matrix
Co = ctrb(A, B);

% Check if the system is controllable by examining the rank
rank_Co = rank(Co);
disp(['Rank of the controllability matrix: ', num2str(rank_Co)])

% If the system is controllable, apply state feedback control
if rank_Co == 4
    % Define the state feedback gain matrix K (using place or a similar method)
    K = place(A, B, [-2 -2.5 -3 -4]);  % Placing poles at desired locations
    
    % Create the closed-loop A matrix (A_cl = A - BK)
    A_cl = A - B * K;
    
    % Simulate the system's response with the feedback applied
    sys_cl = ss(A_cl, B, C, D);  % State-space representation of the closed-loop system
    
    % Check the eigenvalues of the closed-loop system to verify stability
    eig_A_cl = eig(A_cl);
    disp('Eigenvalues of A_cl:');
    disp(eig_A_cl);
    
    % Simulate the system response with initial conditions (starting at an unstable point)
    initial_conditions = [0.1; 0; pi + 0.1; 0];  % Small perturbation from the upright position
    t = 0:0.01:5;  % Time vector for simulation
    
    % Simulate the system's behavior over time
    [y, t, x] = initial(sys_cl, initial_conditions, t);
    
    % Plot the results (cart position and pendulum angle)
    figure;
    subplot(2, 1, 1);
    plot(t, y(:, 1), 'r'); % Cart position
    title('Cart Position');
    xlabel('Time (s)');
    ylabel('Position (m)');
    
    subplot(2, 1, 2);
    plot(t, y(:, 2), 'b'); % Pendulum angle
    title('Pendulum Angle');
    xlabel('Time (s)');
    ylabel('Angle (rad)');
else
    disp('The system is not controllable.');
end
