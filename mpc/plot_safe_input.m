function plot_safe_input(U_safe,title_text, u_mpc, u_s)
    a_x = u_mpc(1);
    v_y = u_mpc(2);
    plot(Polyhedron('ub', [3 1.8], 'lb', [-3 -1.8]), 'Color', 'r');
    hold on;
    plot(U_safe,'Color','b','EdgeAlpha', 0);
    plot(a_x,v_y,'og','markersize',15, 'linewidth', 2);
    if nargin == 4
        plot(u_s(1),u_s(2),'.g','markersize',30);
    end
    title(title_text);
end