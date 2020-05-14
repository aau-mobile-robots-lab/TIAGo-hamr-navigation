function [point,distance] = ClosestPointAndDistance2Polygon(position, poly_x, poly_y)
    distance = inf;
    point = [0,0];
%     i = 0;
    for k = 1:size(poly_x,1)
        if k < size(poly_x,1)
            dist = FindDistanceToClosestPoint(position, [poly_x(k), poly_y(k)], [poly_x(k+1), poly_y(k+1)]);
            p = FindClosestPointOnLine(position, [poly_x(k), poly_y(k)], [poly_x(k+1), poly_y(k+1)]);
            point = if_else(dist<distance, p, point);
            distance = if_else(dist<distance, dist, distance);

        else
            dist = FindDistanceToClosestPoint(position, [poly_x(end), poly_y(end)], [poly_x(1), poly_y(1)]);
            p = FindClosestPointOnLine(position, [poly_x(end), poly_y(end)], [poly_x(1), poly_y(1)]);
            %[distance, i] = if_else(dist<distance, [dist,k], [distance,i])
            point = if_else(dist<distance, p, point);
            distance = if_else(dist<distance, dist, distance);

        end
    end
%     if i == size(poly_x,1)
%         point = FindClosestPointOnLine(position, [poly_x(i), poly_y(i)], [poly_x(1), poly_y(1)]);
%     else
%         point = FindClosestPointOnLine(position, [poly_x(i), poly_y(i)], [poly_x(i+1), poly_y(i+1)]);
%     end
end