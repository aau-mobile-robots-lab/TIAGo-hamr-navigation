function distance = FindDistanceToClosestPoint(position, line_start, line_end)
    distance = norm(position - FindClosestPointOnLine(position, line_start, line_end),2);
end