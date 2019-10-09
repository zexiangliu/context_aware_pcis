function visual3(V,fig)
if(nargin == 1)
    figure;
else
    figure(fig);
end
color = 'r';
V_new = V.copy();
box = Polyhedron("lb",[-inf,-inf,-50],"ub",[inf,inf,50]);
V_new = IntersectPolyUnion(V_new,box);
subplot(221);hold on;
    plot(V_new.slice([1], [25]),'color',color);
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 2.8 -30 30]);
    xlabel('$y_e$', 'interpreter','latex'); ylabel('$h$', 'interpreter','latex');
%     set(gca,"fontsize",12)
%     title('$v_{e,x} = 25 m/s$', 'interpreter','latex')
    set(gca,'fontsize',30)

    subplot(222);hold on;
    plot(V_new.slice([1], [30]),'color',color);
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 2.8 -30 30]);
    xlabel('$y_e$', 'interpreter','latex'); ylabel('$h$', 'interpreter','latex');
%     set(gca,"fontsize",12)
%     title('$v_{e,x} = 30 m/s$', 'interpreter','latex')
    set(gca,'fontsize',30)

    subplot(223);hold on;
    plot(V_new.slice([1], [16]),'color',color);
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 2.8 -30 30]);
    xlabel('$y_e$', 'interpreter','latex'); ylabel('$h$', 'interpreter','latex');
%     set(gca,"fontsize",12)
%     title('$v_{e,x} = 16 m/s$', 'interpreter','latex')
    set(gca,'fontsize',30)

    subplot(224);hold on;
    plot(V_new.slice([1], [25]),'color',color);
    set(gca,'Xdir','reverse','Ydir','reverse')
    axis([-1 2.8 -30 30]);
    xlabel('$y_e$', 'interpreter','latex'); ylabel('$h$', 'interpreter','latex');
%     set(gca,"fontsize",12)
%     title('$v_{e,x} = 25 m/s$', 'interpreter','latex');
    set(gca,'fontsize',30)

    drawnow;
end