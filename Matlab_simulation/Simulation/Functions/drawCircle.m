function drawCircle (x, y, r, color)
    draw_ang=0:0.01:2*pi;
    x_circ = r*cos(draw_ang);
    y_circ = r*sin(draw_ang);
    plot(x+x_circ,y+y_circ,color);
end