function dist_i = GetListOfClosestNObstacleIndex(poly_centroid, position, n_SO)
    distances = [];
    closest_SO = [];
    for k = 1:size(poly_centroid, 1)
        distances = [distances, norm([position(1), position(2)] - poly_centroid(k, 1:2),2)];
    end
    
    [sorted_dist, dist_i] = sort(distances, 'ascend');
end