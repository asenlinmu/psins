glvs
% trj = trjfile('leador');
[nn, ts, nts] = nnts(2, trj.ts);
imuerr = imuseterr([0.01;0; 50;0]*0);
imu = imuadderr(trj.imu, imuerr);
davp = avpseterr([30; 130; 30], 0, 0);
ins = insinit(trj.avp0, ts, davp);
xk = [davp;imuerr.eb;imuerr.db];
len = length(imu);    [avp, xkk] = prealloc(fix(len/nn), 10, 16);
ki = timebar(nn, len, 'Pure inertial navigation processing.');
for k=1:nn:len-nn+1
	k1 = k+nn-1;
	wvm = imu(k:k1, 1:6);  t = imu(k1,7);  
	ins = insupdate(ins, wvm);   %ss.vn = s_avp(k1+1,4:6)';
    TT = .02;
    if mod(t,TT)==0
        ins.vn(3) = trj.avp(k1,6); xk(6) = 0;
        [fk,ft] = kffk(ins, 15);
        xk = (eye(15)+ft*TT+ft*ft*(TT*TT/2))*xk;
        xkk(ki,:) = [xk; t]';
    end
    avp(ki,:) = [ins.avp; t]';
	ki = timebar;
end
avperr = avpcmp(avp, trj.avp);
% insplot(avp);
inserrplot(avperr);
[tt, i1, i2] = intersect(avperr(:,end), xkk(:,end));
err = [avperr(i1,7:8)*glv.Re]-[xkk(i2,7:8)*glv.Re];
figure, plot(xkk(i1,end)-xkk(i1(1),end),err), grid on
figure, plot(xkk(i1,end)-xkk(i1(1),end),[avperr(i1,7:8)*glv.Re]), xygo('dP')
legend('\it\delta L', '\it\delta \lambda')

return;
t1=0:length(err11)-1; t10=(0:length(err101)-1)*0.1;  t50=(0:length(err501)-1)*0.02; 
figure,plot(t1, [err11(:,1),err12(:,1)], t10, [err101(:,1),err102(:,1)], t50, [err501(:,1),err502(:,1)]), xygo('dlat')
legend('1Hz Order1', '1Hz Order2', '10Hz Order1', '10Hz Order2', '50Hz Order1', '50Hz Order2')
figure,plot(t1, [err11(:,2),err12(:,2)], t10, [err101(:,2),err102(:,2)], t50, [err501(:,2),err502(:,2)]), xygo('dlon')
legend('1Hz Order1', '1Hz Order2', '10Hz Order1', '10Hz Order2', '50Hz Order1', '50Hz Order2')
