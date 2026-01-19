function [k1, k2, k3, k4, k5] = estimate_parameters()
    % Load the data
    data = load('U_omega_dynamic.mat').data;

    % Extract signals
    uSig      = data.getElement('Add:1').Values;
    omegaSig  = data.getElement('Mux:1(1)').Values;
    domegaSig = data.getElement('Derivative:1').Values;

    % Ensure same time vector
    t = omegaSig.Time;
    uSig      = resample(uSig, t);
    domegaSig = resample(domegaSig, t);

    u      = uSig.Data(:);
    omega  = omegaSig.Data(:);
    domega = domegaSig.Data(:);

    % =====================================================
    % Regression Matrix (NO ω³ TERM, NO LAGGED DOMEGA)
    % Includes:
    %   u
    %   omega
    %   omega^2
    %   |omega|*omega
    %   sgn(omega)
    % =====================================================

    Phi = [ ...
        u, ...                          % k1
        omega, ...                      % k2
        omega.^2, ...                   % k3
        omega .* abs(omega), ...        % k4
        sign(omega) ...                 % k5
    ];

    % Fit least squares: domega = Phi * params
    params = Phi \ domega;

    k1 = params(1);
    k2 = params(2);
    k3 = params(3);
    k4 = params(4);
    k5 = params(5);

    fprintf('\nEstimated model:\n');
    fprintf(['omega_dot = %.6f*u + %.6f*omega + %.6f*omega^2 + %.6f*|omega|*omega +\n' ...
             '                         %.6f*sgn(omega)\n'], ...
             k1, k2, k3, k4, k5);

    % =====================================================
    % Compute residuals (same logic)
    % =====================================================

    domega_pred = Phi * params;
    residual = domega - domega_pred;

    rmse = sqrt(mean(residual.^2));
    sse  = sum(residual.^2);

    fprintf('\nModel Error:\n');
    fprintf('  RMSE = %.6f\n', rmse);
    fprintf('  SSE  = %.6f\n', sse);

    % =====================================================
    % Forward Simulation of omega(t)  (same as before)
    % =====================================================

    dt = mean(diff(t));
    omega_sim = zeros(size(omega));
    omega_sim(1) = omega(1);

    for k = 2:length(t)
        w = omega_sim(k-1);

        domega_k = ...
            k1*u(k-1) + ...
            k2*w + ...
            k3*(w^2) + ...
            k4*(w*abs(w)) + ...
            k5*sign(w);

        omega_sim(k) = omega_sim(k-1) + domega_k * dt;
    end

    % =====================================================
    % PLOTTING (unchanged)
    % =====================================================

    % Residual plot
    figure;
    plot(t, residual);
    xlabel('Time (s)');
    ylabel('Error in \omega\_dot');
    title('Model Residual');
    grid on;

    % Measured vs simulated omega
    figure;
    plot(t, omega, 'LineWidth', 1.5, 'DisplayName', 'Measured \omega');
    hold on;
    plot(t, omega_sim, '--', 'LineWidth', 1.5, 'DisplayName', 'Simulated \omega');
    xlabel('Time (s)');
    ylabel('\omega (rad/s)');
    title('Measured vs Simulated Motor Speed');
    legend;
    grid on;

end
