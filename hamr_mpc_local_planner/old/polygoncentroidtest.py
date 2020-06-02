import numpy as np

def poligon2centroid(poly_x, poly_y):
    x_mean = np.mean(poly_x)
    y_mean = np.mean(poly_y)
    x = poly_x - x_mean
    y = poly_y - y_mean

    #create shifted matrix for counter clockwise bounderies
    xp = np.append(x[1:], x[0])
    yp = np.append(y[1:], y[0])

    #calculate the twice signed area of the elementary triangle formed by
    #(xi,yi) and (xi+1,yi+1) and the origin.
    a = np.dot(x, yp) - np.dot(xp, y)

    #Sum of the half of these areas
    area = np.sum(a)/2

    if area < 0:
        area = -area

    #calculate centroid of the shifted
    xc = np.sum(np.dot((x+xp), a))/(6*area)
    yc = np.sum(np.dot((y+yp), a))/(6*area)

    #shift back to original place
    centroid_x = xc + x_mean
    centroid_y = yc + y_mean
    centroid_radius = 0

    #calculate radius
    for k in range(poly_x.shape[0]):
        dist = np.linalg.norm(np.array([poly_x[k], poly_y[k]])-np.array([centroid_x, centroid_y]))
        if centroid_radius < dist:
            centroid_radius = dist
    return centroid_x, centroid_y, centroid_radius

poly_x = np.array([3.0, 6.0, 7.0, 4.0])
poly_y = np.array([2.5, 3.0, 5.5, 6.0])
[centroid_x, centroid_y, centroid_r] = poligon2centroid(poly_x, poly_y)

print(np.array([centroid_x, centroid_y, centroid_r]))