function Plot_Control_Input(t, u_cl, v_min, v_max, w_min, w_max)

    figure(101)
    fig = gcf; %Current figure handle
    fig.Color = 'w';
    fig.Units = 'normalized';
    fig.OuterPosition = [0.5 1 0.5 0.5];
    fig.PaperPositionMode = 'auto';
    subplot(2,1,1)
    stairs(t,u_cl(:,1),'b','linewidth',1.5);
    axis([0 t(end) v_min-0.1 v_max+0.1])
    ylabel('$v$ (m/s)', 'interpreter','latex','FontSize', 16)
    
    grid on
    subplot(212)
    stairs(t,u_cl(:,2),'b','linewidth',1.5);
    axis([0 t(end) w_min-pi/16 w_max+pi/16])
    xlabel('time (seconds)', 'interpreter','latex','FontSize', 16)
    ylabel('$\omega$ (rad/s)', 'interpreter','latex','FontSize', 16)
    grid on
end

