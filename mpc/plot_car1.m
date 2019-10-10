function plot_car1(x_e,x)
%     figure(fig);
%     msiz = 200; % marker size
%     malpha = 0.5; % marker alpha
%     fsiz = 12;
%     y_e = x(2);
%     h = x(3);
%     car1 = scatter(x_e, y_e + 4.1, msiz, 'sb',...
%             'MarkerFaceColor', 'b','MarkerEdgeColor','b');
%     car2 = scatter(x_e+h, 4.1, msiz, 's',...
%         'MarkerFaceColor', 'k', 'MarkerEdgeColor','k');
% 
%     text(x_e, 5.5 + y_e,'ego1 wo sp','FontSize',fsiz,...
%         'Color','b','HorizontalAlignment', 'center');
%     text(x_e+h, 4.3,'cau1','FontSize',fsiz,...
%         'Color','k','HorizontalAlignment', 'center');
% 
% %         car1.MarkerFaceAlpha = malpha;
%     car2.MarkerFaceAlpha = malpha;
    im1 = imread('cars/car_blue.png');
    im2 = imread('cars/car_black.png');
    malpha = 1; % marker alpha
    fsiz = 12;
    y_e = x(2);
    h = x(3);
    len = 4;
    wid = 0.4;
    x1 = [x_e-len,x_e+len];
    y1 = [y_e + 4.1+wid,y_e + 4.1-wid];
%     alphachannel = zeros(size(im1,1));
    alphachannel = ~(im1(:,:,1)==255 & im1(:,:,2)==255 & im1(:,:,3)==255);
    car1 = imagesc(x1, y1,im1,'AlphaData', alphachannel);
    
    x2 = [x_e+h-len,x_e+h+len];
    y2 = [4.1+wid,4.1-wid];
    car2 = imagesc(x2, y2, im2,'AlphaData', alphachannel);

    text(x_e, 5.2 + y_e,'ego1 wo sp','FontSize',fsiz,...
        'Color','b','HorizontalAlignment', 'center');
    text(x_e+h, 4.7,'cau1','FontSize',fsiz,...
        'Color','k','HorizontalAlignment', 'center');

    car1.AlphaData = malpha*alphachannel;
    car2.AlphaData = malpha*alphachannel;
        
end