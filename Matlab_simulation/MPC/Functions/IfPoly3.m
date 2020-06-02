function const_vect = IfPoly3(const_vect, poly, pose, rob_diameter)
    closest_point = Closest_point_2_Poly(poly(1:6), 3, pose);
    const_vect = [const_vect; -sqrt((pose(1)-closest_point(1))^2+(pose(2)-closest_point(2))^2) + rob_diameter/2];
end

