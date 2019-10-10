function plot_car2(x_e,x, xl, lead_lane)
%     figure(fig);
%     msiz = 200; % marker size
%     malpha = 0.5; % marker alpha
%     fsiz = 12;
%     y_e = x(2);
%     h = x(3);
%     car1 = scatter(x_e, y_e + 4.1, msiz, 'sb',...
%             'MarkerFaceColor', 'g','MarkerEdgeColor','g');
%     car2 = scatter(x_e+h, 4.1, msiz, 's',...
%         'MarkerFaceColor', 'r', 'MarkerEdgeColor','r');
% 
%     text(x_e, 4.6 + y_e,'ego2 wt sp','FontSize',fsiz,...
%         'Color','b','HorizontalAlignment', 'center');
%     text(x_e+h, 3.8,'cau2','FontSize',fsiz,...
%         'Color','k','HorizontalAlignment', 'center');
% 
%     car1.MarkerFaceAlpha = malpha;
%     car2.MarkerFaceAlpha = malpha;


    im1 = imread('cars/car_green.png');
    im2 = imread('cars/car_red.png');
    malpha = 0.7; % marker alpha
    fsiz = 12;
    y_e = x(2);
    len = 4;
    wid = 0.4;
    x1 = [x_e-len,x_e+len];
    y1 = [y_e + 4.1+wid,y_e + 4.1-wid];
    alphachannel = ~(im1(:,:,1)==255 & im1(:,:,2)==255 & im1(:,:,3)==255);
    car1 = imagesc(x1, y1,im1,'AlphaData', 0);
    car1.AlphaData = malpha*alphachannel;

    for i = 1:length(lead_lane)
        h = xl(2*i-1);
        x2 = [x_e+h-len,x_e+h+len];
        y2 = [4.1+wid+lead_lane(i)*1.8,4.1-wid+lead_lane(i)*1.8];
        car2 = imagesc(x2, y2, im2,'AlphaData', 0);

        text(x_e, 4.6 + y_e,'ego2 wt sp','FontSize',fsiz,...
            'Color','g','HorizontalAlignment', 'center');
        text(x_e+h, 3.6,'cau2','FontSize',fsiz,...
            'Color','r','HorizontalAlignment', 'center');        
        car2.AlphaData = malpha*alphachannel;
    end
end