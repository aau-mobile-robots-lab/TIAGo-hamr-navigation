clear all
close all
clc

line_start = [0, 0]
line_end = [4, 4]
position = [4, 0]

plot([line_start(1), line_end(1)], [line_start(2), line_end(2)], 'b')
hold on
plot(position(1), position(2), 'r*')
hold on

closestpoint = FindClosestPointOnLine(position, line_start, line_end);
plot(closestpoint(1), closestpoint(2), 'g*')
distance = FindDistanceToClosestPoint(position, line_start, line_end)