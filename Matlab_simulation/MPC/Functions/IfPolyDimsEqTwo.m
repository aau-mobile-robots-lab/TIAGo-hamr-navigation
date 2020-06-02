function const_vect = IfPolyDimsEqTwo(const_vect,poly,pose,rob_diameter)
    closest_point = FindClosestPointOnLine([pose(1), pose(2)], [poly(1), poly(2)], [poly(3), poly(4)]);
    const_vect = [const_vect; -sqrt(max(0.0000001,(pose(1)-closest_point(1))^2+(pose(2)-closest_point(2))^2)) + rob_diameter/2];
end

