function fitted_path = BuiltIn_trajectory_fitting(path, n_order)
distance_vector = diff(path);
theta = [0;cumsum(sqrt(distance_vector(:,1).^2 + distance_vector(:,2).^2))];

coef_x = polyfit(theta, path(:,1),n_order);
coef_y = polyfit(theta, path(:,2),n_order);

param = linspace(0, theta(end))
x_fitted = polyval(coef_x, param);
y_fitted = polyval(coef_y, param);

fitted_path = [x_fitted', y_fitted'];