function [centroid_x, centroid_y, centroid_radius] = CalculatePolygonCentroid(poly_x, poly_y)
    
    % temporarily shift data to mean of vertices for improved accuracy
    x_mean = mean(poly_x);
    y_mean = mean(poly_y);
    x = poly_x - x_mean;
    y = poly_y - y_mean;
    
    %create shifted matrix for counter clockwise bounderies
    xp = x([2:end 1]);
    yp = y([2:end 1]);
    
    %calculate the twice signed area of the elementary triangle formed by
    %(xi,yi) and (xi+1,yi+1) and the origin.
    a = x.*yp - xp.*y;
    
    %Sum of the half of these areas
    A = sum(a)/2;
    
    if A < 0
        A = -A;
    end
    
    %calculate centroid of the shifted 
    xc = sum((x+xp).*a)/(6*A);
    yc = sum((y+yp).*a)/(6*A);
    
    %shift back to original place
    centroid_x = xc + x_mean;
    centroid_y = yc + y_mean;
    centroid_radius = 0;
    
    %calculate radius
    for k = 1:size(poly_x)
        dist = norm([poly_x(k), poly_y(k)]-[centroid_x, centroid_y],2);
        if centroid_radius<dist
            centroid_radius = dist;
        end
    end
end