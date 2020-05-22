clear all
close all
clc

figure(1)
fig = gcf; %Current figure handle
fig.Color = 'w';
fig.Units = 'pixel';
fig.OuterPosition = [0 0 1000 1000];
fig.PaperPositionMode = 'auto';

SO_polygon(1).point(1).x = {3.0}; SO_polygon(1).point(1).y = {2.5};
SO_polygon(1).point(2).x = {6.0}; SO_polygon(1).point(2).y = {3.0};
SO_polygon(1).point(3).x = {7.0}; SO_polygon(1).point(3).y = {5.5};
SO_polygon(1).point(4).x = {4.0}; SO_polygon(1).point(4).y = {6.0};
[obs, obs_sizes] = SO_struct2Matrix(SO_polygon);
[poly, poly_size] = MakeSOVectorAndDims(SO_polygon);

polycentroid = CalculatePolygonCentroid(obs(:,1), obs(:,2));
pol_cent_x = polycentroid(1);
pol_cent_y = polycentroid(2);
radius = polycentroid(3) + 0.3;

draw_ang=0:0.05:2*pi;
pose_circ_x = pol_cent_x+radius*cos(draw_ang);
pose_circ_y = pol_cent_y+radius*sin(draw_ang);
const_vect=[];

for k = 1:size(pose_circ_x,2)
    plot([obs(:,1);obs(1,1)], [obs(:,2);obs(1,2)], 'b-*')
    hold on
    plot(pose_circ_x(k), pose_circ_y(k), 'ro')
    hold on
    drawCircle(pol_cent_x, pol_cent_y, radius, 'k--')
    hold on
    
    point = IfPolyHasArea(const_vect, poly, poly_size, [pose_circ_x(k), pose_circ_y(k)], 0);
    %[point, distance] = FindClosestPointandDistance2Polygon([pose_circ_x(k), pose_circ_y(k)], SO_polygon);
    
    plot(point(1), point(2), 'go')
    hold on
    plot([pose_circ_x(k), point(1)], [pose_circ_y(k), point(2)], '--g')
    axis([0 8 0 8])
    pause(0.1)
    hold off
    
    drawnow
end

