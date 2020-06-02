function point = Closest_point_2_Poly(poly, dim, pose)
    poly_x = poly(1:2:2*dim-1);
    poly_y = poly(2:2:2*dim);
    
    point = FindClosestPointOnLine([pose(1), pose(2)], [poly_x(1), poly_y(1)], [poly_x(2), poly_y(2)]);
    distance = norm(pose - point,2);
    
    for k = 2:(dim-1)
        p = FindClosestPointOnLine([pose(1), pose(2)], [poly_x(k), poly_y(k)], [poly_x(k+1), poly_y(k+1)]);
        dist = norm(pose - p,2);
        
        point = if_else(dist<distance, p, point);
        distance = if_else(dist<distance, dist, distance);
    end
    
    p = FindClosestPointOnLine([pose(1), pose(2)], [poly_x(dim), poly_y(dim)], [poly_x(1), poly_y(1)]);
    dist = norm(pose - p,2);
    
    point = if_else(dist<distance, p, point);
end