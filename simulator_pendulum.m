function x_new = simulator_pendulum(x0, u, Ad, Bd)
    % initialize output
    traj_points = length(u); % number of trajectory points
    x_new = zeros(2, traj_points); % 2 states and N traj poits.
    x_new(:,1) = x0; % initialize first trajectory point as x0.

    % Output
    for i = 1:(traj_points-1)
        x_new(1,i) = mod(x_new(1,i), 2*pi); % wrap around [0, 2pi]
        x_new(:, i+1) = Ad*x_new(:,i) + Bd*u(i);
    end
end