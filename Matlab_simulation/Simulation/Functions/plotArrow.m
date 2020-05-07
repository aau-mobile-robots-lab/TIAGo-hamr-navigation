function plotArrow(x, y, th, arrow_length, arrow_h, arrow_w, color)
    [dx, dy] = pol2cart(th, arrow_length-arrow_h);
    x_ar = x + dx;
    y_ar = y + dy; 
    plot([x x_ar], [y y_ar], color, 'Linewidth', 2.5);
    hold on;
    x_tri = [x_ar+arrow_h*cos(th), x_ar+(arrow_w/2)*cos((pi/2)-th), x_ar-(arrow_w/2)*cos((pi/2)-th)];
    y_tri = [y_ar+arrow_h*sin(th), y_ar-(arrow_w/2)*sin((pi/2)-th), y_ar+(arrow_w/2)*sin((pi/2)-th)];
    fill(x_tri, y_tri, color);
end