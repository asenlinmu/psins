gbdvars
t = 0:60:3600*24;  len = length(t);
% [eph, ephToeIdx] = rnx302n('cubb1870.15c');
tp = cal2bdt([2015 07 05 24 00 0]); tp;
timebar(1,len);  satpos = zeros(len,15); satvel = satpos;
kk = 1;
prns = 100+[1 2 3 4 5];
for k = 1:len
    idx = findEph(tp, prns, ephToeIdx);  ephi = eph(idx,:);
    [satPos, clkCorr, satVel] = bdSatPosVel(tp*ones(size(idx'))+t(k), epha2s(ephi,'C'));
    satpos(kk,:) = reshape(satPos, 1, 15);
    satvel(kk,:) = reshape(satVel, 1, 15);  kk = kk+1;
    timebar;
end
for k=1:15; satpos(:,k) = satpos(:,k)-satpos(1,k); end
myfigure, 
subplot(321), plot(t/3600,satpos(:,1:3:end)), xygo('t / h','\Delta X / m')
legend('NO.1', 'NO.2', 'NO.3', 'NO.4', 'NO.5'),title('2015-07-05日BD-GEO卫星在ECEF系下的相对位置变化')
subplot(323), plot(t/3600,satpos(:,2:3:end)), xygo('t / h','\Delta Y / m')
subplot(325), plot(t/3600,satpos(:,3:3:end)), xygo('t / h','\Delta Z / m')
subplot(322), plot(t/3600,satvel(:,1:3:end)), xygo('t / h','Vx / m / s'), title('BD-GEO卫星的速度')
subplot(324), plot(t/3600,satvel(:,2:3:end)), xygo('t / h','Vy / m / s')
subplot(326), plot(t/3600,satvel(:,3:3:end)), xygo('t / h','Vz / m / s')
myfigure
E = sqrt(satpos(:,1:3:end).^2+satpos(:,2:3:end).^2); N = satpos(:,3:3:end);
plot(E/4.5e7/glv.deg,N/4.5e7/glv.deg);  xygo('E / \circ', 'N / \circ');
legend('NO.1', 'NO.2', 'NO.3', 'NO.4', 'NO.5'),title('2015-07-05日BD-GEO卫星在ECEF系下的相对位置变化')
return
v = diff(satpos(:,:)); v = [v(1,:); v]/60;
figure, plot(t/3600,v(:,1:3:end)), xygo('t / h','\Delta V / m')

