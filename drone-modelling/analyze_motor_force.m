load("F_omega.mat");
signal1 = data.getElement(1).Values.Data;   % Extract the first 1x1 signal
signal2 = data.getElement(2).Values.Data;   % Extract the second 1x1 signal
F_motor = signal1 .* -1.0;
w_motor = signal2 .* 1.0;

p = polyfit(w_motor, F_motor, 2)
% Generate the polynomial values for the fitted model
F_fit = polyval(p, w_motor);

figure;
hold on;
plot(w_motor, F_motor, 'o', 'DisplayName', 'Data Points'); % Plot the original data points
plot(w_motor, F_fit, '-', 'DisplayName', 'Polynomial Fit'); % Plot the polynomial fit
xlabel('F_motor');
ylabel('w_motor');
title('Polynomial Fit of w_motor vs F_motor');
legend show;
grid on;
hold off;

figure;
hold on;
plot(F_motor, w_motor, 'o', 'DisplayName', 'Data Points'); % Plot the original data points
plot(F_motor, 166.38 * sqrt(F_motor), '-', 'DisplayName', '106.8739 * sqrt(F_motor)'); % Plot the transformed data
xlabel('F_motor');
ylabel('w_motor');
title('Plot of w_motor vs F_motor and 106.8739 * sqrt(F_motor)');
legend show;
grid on;
hold off;