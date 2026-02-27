%% forward
glvs
kftypedef('leador_ins_gps_19');
ts = 0.005; nn = 20;  nts = nn*ts;  % ×ÓÑùÊý
% imu = bfile('imu.bin',7); gps = bfile('gps.bin',15); gpsfind(gps(:,2));
% att0 = align_i0(imu(600/ts:1000/ts,2:7), gps(1,3:5)', ts);
ss = sins([att0',[0,0,0],gps(1,3:5)], ts);
kf = kf_init(nts);
kk = 1000/ts; ki = 1;
len = 1000;  res = zeros(len,13);  xkpk = zeros(len,2*kf.n); xfb = zeros(len,kf.n);
timebar(nts,len,'SINS/GPS forward simulation.');
for k=kk+2:nn:(kk+len/ts)
    k1 = k+nn-1; wvm = imu(k:k1,2:7);
    ss = sins(ss, wvm);
    Ft = kf_Ft(ss); M5 = Ft(7:9,4:6); kf.Phikk_1 = kf.I + Ft*nts;
    kf = kalman(kf);
    if mod(k1,1/ts)==1
        kgps = gpsfind(imu(k1,1)+16); % if(kgps>0)
        posS = gps(kgps,3:5)';
        if gps(kgps, 6)>0
            kf.Hk(:,16:19) = [-M5*ss.Cnb, -M5*ss.vn];
            kf = kalman(kf, ss.pos-posS, 'M');
            [kf, ss] = kf_feedback(kf, ss, 1, 'avped');
        end
        res(ki,:) = [ss.avp; posS; imu(k1,1)]'; 
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk)]; xfb(ki,:) = kf.xfb'; ki = ki+1;
    end
    timebar;
end
% navplot(res(:,1:9),res(:,end))
% kf_plot(xkpk,res(:,end))
%% reverse
glv.wie = -glv.wie; ss.vn = -ss.vn;
kk = k1; ki = 1; ires = zeros(len,13);  ixkpk = zeros(len,2*kf.n); ixfb = zeros(len,kf.n);
timebar(nts,len,'SINS/GPS reverse simulation.');
for k=kk:-nn:(kk-len/ts)
    k1 = k-nn+1; wvm = imu(k:-1:k1,2:7); wvm(:,1:3) = -wvm(:,1:3);
    ss = sins(ss, wvm);
    Ft = kf_Ft(ss); M5 = Ft(7:9,4:6); kf.Phikk_1 = kf.I + Ft*nts;
    kf = kalman(kf);
    if mod(k1,1/ts)==2
        kgps = gpsfind(imu(k1-1,1)+16); % if(kgps>0)
        posS = gps(kgps,3:5)';
        if gps(kgps, 6)>0
            kf.Hk(:,16:19) = [-M5*ss.Cnb, M5*ss.vn]; % dt sign change vs forward
            kf = kalman(kf, ss.pos-posS, 'M');
           [kf, ss] = kf_feedback(kf, ss, 1, 'avp');
        end
        ires(ki,:) = [ss.avp; posS; imu(k1-1,1)]'; 
        ixkpk(ki,:) = [kf.xk; diag(kf.Pxk)]; ixfb(ki,:) = kf.xfb'; ki = ki+1;
    end
    timebar;
end
% navplot([[ires(:,1:3),-ires(:,4:6),ires(:,7:9)]],ires(:,end))
% kf_plot(ixkpk,ires(:,end))
navplot([res(:,1:9);[ires(:,1:3),-ires(:,4:6),ires(:,7:9)]],[res(:,end);ires(:,end)])
kf_plot([xkpk;ixkpk],[res(:,end);ires(:,end)])
%% fusion
avp1 = res(:,[1:9,13]); avp2 = ires(:,[1:9,13]);
[avp, p] = avpfusion(avp1, xkpk(:,1:9), xkpk(:,20:28), avp2, ixkpk(:,1:9), ixkpk(:,20:28));

[pos1, pp1] = leverpos(avp1, xkpk(:,1:19), xkpk(:,20:end));
[pos2, pp2] = leverpos(avp2, ixkpk(:,1:19), ixkpk(:,20:end));
figure, 
subplot(321)
plot(res(:,13),[pos1(:,1)-res(:,10)]*glv.Re,ires(:,13),[pos2(:,1)-ires(:,10)]*glv.Re), xlgo
subplot(323)
plot(res(:,13),[pos1(:,2)-res(:,11)]*glv.Re,ires(:,13),[pos2(:,2)-ires(:,11)]*glv.Re), xlgo
subplot(325)
plot(res(:,13),[pos1(:,3)-res(:,12)],ires(:,13),[pos2(:,3)-ires(:,12)]), xlgo
subplot(322), plot(res(:,13),sqrt(pp1(:,1))*glv.Re,ires(:,13),sqrt(pp2(:,1))*glv.Re), xlgo
subplot(324), plot(res(:,13),sqrt(pp1(:,2))*glv.Re,ires(:,13),sqrt(pp2(:,2))*glv.Re), xlgo
subplot(326), plot(res(:,13),sqrt(pp1(:,3)),ires(:,13),sqrt(pp2(:,3))), xlgo