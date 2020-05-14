function [line_start, line_end] = FindPolygonClosestEdgePair(position, poly_x, poly_y)
    closest = inf;
    for k = 1:size(poly_x)
        dist = norm(position - [poly_x(k), poly_y(k)],2)
        if dist<closest
            i = k;
            closest = dist;
        end
    end
    
    line_start = [poly_x(i), poly_y(i)];
    
    if i == 1
        if norm(position - [poly_x(2), poly_y(2)],2)<norm(position - [poly_x(end), poly_y(end)],2)
            line_end = [poly_x(2), poly_y(2)];
        else
            line_end = [poly_x(end), poly_y(end)];
        end
    elseif i == size(poly_x)
        if norm(position - [poly_x(i-1), poly_y(i-1)],2)<norm(position - [poly_x(1), poly_y(1)],2)
            line_end = [poly_x(i-1), poly_y(i-1)];
        else
            line_end = [poly_x(1), poly_y(1)];
        end
    else
        if norm(position - [poly_x(i-1), poly_y(i-1)],2)<norm(position - [poly_x(i-1), poly_y(i-1)],2)
            line_end = [poly_x(i+1), poly_y(i+1)];
        else
            line_end = [poly_x(i-1), poly_y(i-1)];
        end
    end
end