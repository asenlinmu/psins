
h = 0.01;
y = [0; 1];
xy = zeros(100,3);
for k = 1:1000
    x = (k-1)*h;
    y = rgkt4(@testfun, [x, x+h/2, x+h], y, h);
    xy(k,:) = [x+h; y]';
end
subplot(211), plot(xy(:,1), [xy(:,2)- (1-cos(xy(:,1)))]), grid on
subplot(212), plot(xy(:,1), [xy(:,3)- exp(xy(:,1))]), grid on