clear all
close all
clc

obs1 = [0.0, 1.0]
obs2 = [1.0, 2.0;
        1.0, 3.0]
obs3 = [1.0, 0.0;
        1.5, 1.0;
        3.0, 0.0;
        2.0, -3.0;
        1.5, -2.0]

obs_sizes = [size(obs1,1), size(obs2,1), size(obs3,1)];
n_obs = 3; %size(obs_sizes, 2);
all_obs = [obs1; obs2; obs3];
pos = 1;
centroid = [];
centroid_x = 0; centroid_y = 0; centroid_r = 0;
poly_x = []; poly_y = [];

for k = 1:n_obs
    if obs_sizes(k) < 2
        centroid = [centroid; all_obs(pos,1), all_obs(pos,2), 0];
        plot(all_obs(pos,1), all_obs(pos,2), 'r*')
        hold on
        pos = pos+1;
    elseif obs_sizes(k) == 2
        centroid_x = (all_obs(pos,1) + all_obs(pos+1,1))/2;
        centroid_y = (all_obs(pos,2) + all_obs(pos+1,2))/2;
        centroid_r = norm(all_obs(pos,1:2)-all_obs(pos+1,1:2))/2;
        centroid = [centroid; centroid_x, centroid_y, centroid_r];
        plot(all_obs(pos:pos+1,1), all_obs(pos:pos+1,2), 'g-*')
        hold on
        plot(centroid_x, centroid_y, 'g*');
        hold on
        drawCircle(centroid_x, centroid_y, centroid_r,'--g');
        hold on
        pos = pos+2;
    else
        poly_x = all_obs(pos:pos+obs_sizes(k)-1,1);
        poly_y = all_obs(pos:pos+obs_sizes(k)-1,2);
        [centroid_x, centroid_y, centroid_r] = CalculatePolygonCentroid(poly_x, poly_y);
        centroid = [centroid; centroid_x, centroid_y, centroid_r];
        plot([poly_x; poly_x(1)], [poly_y; poly_y(1)], 'b-*')
        hold on
        plot(centroid_x, centroid_y, 'b*');
        hold on
        drawCircle(centroid_x, centroid_y, centroid_r,'--b');
        hold on
        pos = pos+obs_sizes(k);
    end
end


%drawCircle(centroid_x, centroid_y, centroid_radius, 'k')
