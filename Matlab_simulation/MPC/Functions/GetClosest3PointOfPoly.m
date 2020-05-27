function obs_poses = GetClosest3PointOfPoly(SO_polygon, pose)
    obs_poses = [];
    distances = [];
    obs_size = size(SO_polygon.point, 2);
    if obs_size == 1
        obs_poses = [SO_polygon.point(1).x{:}, SO_polygon.point(1).y{:},0,0,0,0];
    elseif obs_size == 2
        obs_poses = [SO_polygon.point(1).x{:}, SO_polygon.point(1).y{:}, ...
                    SO_polygon.point(2).x{:}, SO_polygon.point(2).y{:},0,0];
    elseif obs_size == 3
        for i = 1:3
            obs_poses = [obs_poses, [SO_polygon.point(i).x{:}, SO_polygon.point(i).y{:}]];
        end
    else
        for i = 1:obs_size
            dist = norm([pose(1), pose(2)] - [SO_polygon.point(i).x{:}, SO_polygon.point(i).y{:}], 2);
            distances = [distances, dist];
        end
        [sorted_dist, dist_i] = sort(distances, 'ascend');
        for i = 1:3
            obs_poses = [obs_poses, [SO_polygon.point(dist_i(i)).x{:}, SO_polygon.point(dist_i(i)).y{:}]];
        end
    end
end