glvs
% [imu, avp0, ts] = imufile('leador');
% gps = avpfile('leador');
[nn, ts, nts] = nnts(2, ts);
davp = avpseterr([30; 30; 3], 0, 0);
ins = insinit(avp0, ts, davp);
xk = [davp;zeros(6,1)];
len = length(imu);    [avp, xkk] = prealloc(fix(len/nn), 10, 16);
ki = timebar(nn, len, 'Pure inertial navigation processing.');
for k=1:nn:len-nn+1
	k1 = k+nn-1;
	wvm = imu(k:k1, 1:6);  t = imu(k1,7);  
	ins = insupdate(ins, wvm);   %ss.vn = s_avp(k1+1,4:6)';
    if mod(t,1)==0
        ins.vn(3) = gps(gps(:,10)==t,6);
    end
    TT = 1;
    if mod(t,TT)==0
        [fk,ft] = kffk(ins, 15);
        xk = (eye(15)+ft*TT)*xk;
    end
    xkk(ki,:) = [xk; t]';
	avp(ki,:) = [ins.avp; t]';
	ki = timebar;
end
avperr = avpcmp(avp, gps);
% navplot(avp);
inserrplot(avperr);
[tt, i1, i2] = intersect(avperr(:,end), xkk(:,end));
xx = xkk(i2,:);
figure, plot(xx(:,end)-xx(1,end),[avperr(:,7:8)*glv.Re]- ...
                [xx(:,7:8)*glv.Re]), grid on
figure, plot(xx(:,end)-xx(1,end),... 
                [[avperr(:,7:8)*glv.Re,avperr(:,9)], ...
                [xx(:,7:8)*glv.Re,xx(:,9)]]), grid on