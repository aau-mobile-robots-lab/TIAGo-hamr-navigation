clear all
close all
clc

SO_polygon(1).point(1).x = {0.0}; SO_polygon(1).point(1).y = {3.0};
SO_polygon(2).point(1).x = {4.0}; SO_polygon(2).point(1).y = {0.0};
SO_polygon(2).point(2).x = {4.0}; SO_polygon(2).point(2).y = {1.8};
SO_polygon(3).point(1).x = {4.0}; SO_polygon(3).point(1).y = {1.8};
SO_polygon(3).point(2).x = {6.0}; SO_polygon(3).point(2).y = {1.8};
SO_polygon(4).point(1).x = {3.0}; SO_polygon(4).point(1).y = {2.5};
SO_polygon(4).point(2).x = {6.0}; SO_polygon(4).point(2).y = {3.0};
SO_polygon(4).point(3).x = {7.0}; SO_polygon(4).point(3).y = {5.5};
SO_polygon(4).point(4).x = {4.0}; SO_polygon(4).point(4).y = {6.0};

position =[8, 1];

plot([obs(:,1);obs(1,1)], [obs(:,2);obs(1,2)], 'b-*')
hold on
plot(position(1), position(2), 'r*')
hold on


%[e1, e2] = FindPolygonClosestEdgePair(position,size,obs(:,2))
%This is not the proper way to go

[point, distance] = ClosestPointAndDistance2Polygon(position,obs(:,1),obs(:,2));
plot(point(1), point(2), 'g*')
hold on
plot([position(1), point(1)], [position(2), point(2)], '--g')

distance

