function plot_road(x_e,t)
%     figure(fig);
    % road
    
    title(['t = ', num2str(t)]);
    hold on    
% single lane
%     plot([-200 4000],[3.2,3.2],'k-','LineWidth',3);

% double lane
    plot([-200 4000],[1.4,1.4],'k-','LineWidth',3);
    plot([-200 4000],[3.2,3.2],'k--','LineWidth',3);
    plot([-200 4000],[5,5],'k--','LineWidth',3);
    plot([-200 4000],[6.8,6.8],'k-','LineWidth',3);

    
    axis([x_e-100 x_e+100 1.2 7])
    set(gca,'ytick',[])
end