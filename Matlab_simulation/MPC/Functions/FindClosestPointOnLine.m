function closest_point = FindClosestPointOnLine(position, line_start, line_end)
    a2p = position - line_start;
    a2b = line_end - line_start;
    sq_a2b = a2b(1)^2+a2b(2)^2;
    a2p_dot_a2b = a2p(1)*a2b(1)+a2p(2)*a2b(2);
    diff = max(0, min(1, max(0.001, a2p_dot_a2b/sq_a2b)));
    closest_point = line_start+a2b.*diff;
end


