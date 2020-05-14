clear all
close all
clc

addpath('/Users/reiserbalazs/Documents/MATLAB/casadi-osx-matlabR2015a-v3.5.1')
import casadi.*

obs =  [1.0, 0.0;
        1.5, -2.0;
        2.0, -3.0;
        3.0, 0.0;
        1.5, 1.0]

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

