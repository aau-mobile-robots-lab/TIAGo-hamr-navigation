path = [0.1, 0.2;
        0.2, 0.2;
        0.3, 0.2;
        0.4, 0.2;
        0.5, 0.3;
        0.6, 0.4;
        0.7, 0.5;
        0.8, 0.6;
        0.9, 0.7];

p = polyfit(path(:,1), path(:,2), size(path, 1))
y = polyval(p,path(:,1))
plot(path, y)
       