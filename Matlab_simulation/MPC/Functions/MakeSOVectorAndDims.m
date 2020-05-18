function [SO_vector, SO_dims] = MakeSOVectorAndDims(SO_polygon)
    SO_vector = [];
    SO_dims = [];
    for k = 1:size(SO_polygon, 2)
        SO_dims = [SO_dims, size(SO_polygon(k).point, 2)];
        for i = 1:size(SO_polygon(k).point, 2)
            SO_vector = [SO_vector, [SO_polygon(k).point(i).x{:}, SO_polygon(k).point(i).y{:}]];
        end
    end
end