%        X    Y
path = [0.0, 0.0;
        0.1, 0.1;
        0.2, 0.2;
        0.3, 0.3;
        0.4, 0.3;
        0.5, 0.3;
        0.6, 0.3;
        0.7, 0.2;
        0.8, 0.1;
        0.9, 0.0;
        1.0, -.1;
        1.1, -.1;
        1.2, -.1];

n_order = 7;
distance_vector = diff(path);
theta = [0;cumsum(sqrt(distance_vector(:,1).^2 + distance_vector(:,2).^2))];

coef_x = polyfit(theta, path(:,1),n_order);
coef_y = polyfit(theta, path(:,2),n_order);

param = linspace(0, theta(end))
x_fitted = polyval(coef_x, param);
y_fitted = polyval(coef_y, param);

figure(2)
plot(x_fitted, y_fitted)
hold on
plot(path(:,1), path(:,2))
       