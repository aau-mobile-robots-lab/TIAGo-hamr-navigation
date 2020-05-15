function [obs_poses, obs_sizes] = SO_struct2Matrix (SO_polygon)
    obs_poses = [];
    obs_sizes = [];
    for k = 1:size(SO_polygon, 2)
        obs_sizes = [obs_sizes, size(SO_polygon(k).point, 2)];
        for i = 1:size(SO_polygon(k).point, 2)
            obs_poses = [obs_poses; [SO_polygon(k).point(i).x{:}, SO_polygon(k).point(i).y{:}]];
        end
    end
end