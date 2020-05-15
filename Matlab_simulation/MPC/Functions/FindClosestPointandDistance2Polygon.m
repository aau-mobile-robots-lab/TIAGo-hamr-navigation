function [point, distance] = FindClosestPointandDistance2Polygon(position, SO_polygon)
    distance = inf;

    i_end = size(SO_polygon.point,2);
    for k = 1:i_end-1
        line_start = [SO_polygon.point(k).x{:}, SO_polygon.point(k).y{:}];
        line_end = [SO_polygon.point(k+1).x{:}, SO_polygon.point(k+1).y{:}];
        p = FindClosestPointOnLine(position, line_start, line_end);
        dist = norm(position - p,2);
        if dist<distance
            distance = dist;
            point = p;
        end
    end
    
    %Check the edge between the last and first point of the polygon
    line_start = [SO_polygon.point(i_end).x{:}, SO_polygon.point(i_end).y{:}];
    line_end = [SO_polygon.point(1).x{:}, SO_polygon.point(1).y{:}];
    p = FindClosestPointOnLine(position, line_start, line_end);
    dist = norm(position - p,2);
    if dist<distance
        distance = dist;
        point = p;
    end
end