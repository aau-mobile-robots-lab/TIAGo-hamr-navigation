function PlotRobotPose(X, Y)
    figure(102)
    fig = gcf; %Current figure handle
    fig.Color = 'w';
    fig.Units = 'pixel';
    fig.OuterPosition = [0 0 1000 1000];

    plot(X,Y,'b','linewidth',1.5);
    axis equal
    %axis([0 t(end) w_min-pi/16 w_max+pi/16])
    xlabel('X position (m)', 'interpreter','latex','FontSize', 16)
    ylabel('Y position (m)', 'interpreter','latex','FontSize', 16)
    grid on
end

