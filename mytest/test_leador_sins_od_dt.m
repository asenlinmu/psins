glvs
navtypedef('leador_ins_od_dt');
ts = 0.01; nn = 2;  nts = nn*ts;  % ×ÓÑùÊý
% imu = bfile('imu.bin',7); od = bfile('od.bin',1); gps = bfile('gps.bin',15); 
% att = align_i0(imu(3/ts:600/ts,2:7), gps(1,3:5)', ts);
ss = sins(a2qnb(att), zeros(3,1), gps(1,3:5)', ts);  pos = ss.pos;
kf = kf_init(nts);
drInst = [-0.5*glv.deg; 1.5*glv.deg; 0.0016]; prj = drPrj(drInst);
ss1 = DR(a2qnb(att), gps(1,3:5)', drInst, ts);
SVins = zeros(3,1); SVod = SVins; SMod = zeros(3); SMlv = SMod; Ian = SVins;
kk = 600/ts; ki = 1;
len = 4000;  err = zeros(len,21);  resk = zeros(len,2*kf.n);
timebar(nts,len,'SINS/DR simulation.');
for k=kk+1:nn:(kk+len/ts)
    k1 = k+nn-1;
    imui = imu(k:k1,2:7); ds = sum(od(k:k1));
    ss = sins(ss, imui);  ss1 = DR(ss1, imui, ds); pos = pos + vn2dpos(ss.eth, ss.vn, nts);
    dSb = prj*ds; dSn = ss.Cnb*dSb;
    SVins = SVins + ss.vn*nts; SVod = SVod + dSn; 
    SMod = SMod + ss.Cnb*askew(dSb); SMlv = SMlv + ss.Cnb*askew(ss.wb*nts);
    Ian = Ian + (ss.Cnb*ss.fb+ss.eth.gn)*nts;
    Ft = kf_Ft(ss);   kf.Phikk_1 = kf.I + Ft*nts;
    kf = kalman(kf);
    if mod(k1,100)==0
        SMod(:,2) = SVod;
        kf.Hk = [askew(-SVod), glv.I33*100*ts, zeros(3,9), -SMod, -SMlv, Ian];
        kf = kalman(kf, SVins-SVod, 'M');
        err(ki,:) = [ss.avp', gps(k1*ts,3:5), q2att(ss1.qnb)', ss1.pos', pos']; 
        resk(ki,:) = [kf.xk; diag(kf.Pxk)]; ki = ki+1;
        SVins = zeros(3,1); SVod = SVins; SMod = zeros(3); SMlv = SMod; Ian = SVins;
        if k1<(28000+600)*100
            ss.vn = ss.vn-kf.xk(4:6); 
            ss.pos = ss.pos-kf.xk(7:9); kf.xk(4:9) = 0;
        end
    end
    timebar;
end
insplot(err, 1, 'avp'); hold on, plot(err(:,11)/glv.deg, err(:,10)/glv.deg, 'r');
	plot(err(:,17)/glv.deg, err(:,16)/glv.deg, 'm');
	plot(err(:,20)/glv.deg, err(:,19)/glv.deg, 'g');
figure, plot([err(:,7:8)-err(:,10:11)]*glv.Re); grid on
    hold on, plot([err(:,19:20)-err(:,10:11)]*glv.Re, 'g'); grid on
% naverrplot(resk(:,1:kf.n), 1, 'avped_od_dt');
% naverrplot(sqrt(resk(:,(kf.n+1):end)), 1, 'avped_od_dt');