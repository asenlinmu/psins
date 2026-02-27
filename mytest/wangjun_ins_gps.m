glvs
if ~exist('imu', 'var');
%     imu = load('imu_13050103.txt');
%     imu1 = [imu(:,2:4)*0.3814697265625e-5, imu(:,5:7)*0.6103515625000e-4];
%     imu1 = rfu(imu1, 'frd'); % CVect3(wm.j, wm.i, -wm.k); vm = CVect3(vm.j, vm.i, -vm.k); 
%     load sbet_data_imu_frame;
%     gps = load('mgps_13050103.pos');
%     gps = [ gps(:,1:2), dms2r(gps(:,3:5)), dms2r(gps(:,6:8)), gps(:,9:end) ];
%     attr = [data_sbet_imu_frame(:,2:4), data_sbet_imu_frame(:,1)];
%     imu = [imu1, imu(:,1)];
%     save imu_gps imu gps attr -v6
    load imu_gps;
end
kftypedef('test_SINS_GPS_19');
[nn, ts, nts] = nnts(4, 0.005);
kstart = 1/ts;
kimu = find(imu(:,7)==attr(kstart,4),1);
kgps = find(gps(:,2)==fix(imu(kimu,7))+16,1);
att0 = attr(kstart,1:3)';
pos0 = gps(kgps,3:5)'; vn0 = pp2vn(pos0, gps(kgps+1,3:5)');
ins = insinit([att0;vn0;pos0], ts);
imuerr = imuSetErr([0.1, 0.01, 100, 10]*10);
avpErr = [ [1;1;1]*glv.deg; [1;1;1]; [[1;1]/glv.Re;1] ];
lever = [1;1;1];
dT = 0.01;
kf = kfinit(nts, avpErr, imuerr, lever, dT);
len = 3200/ts; 
res = zeros(len/nn,10); err = zeros(len/nn,2*kf.n+1);
kk = timebar(nts, len*ts);
for k=kimu+1:nn:len+kimu
    knn = k+nn-1;
    wvm = imu(k:knn, 1:6);  pos_1 = ins.pos;
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins, kf.fkno);
    kf = kfupdate(kf);
    n = floor(imu(knn,7)); t1 = imu(knn,7)-n; t0 = n-imu(knn-nn,7); T = t0+t1;
    if t0*t1>0 || t1==0
        kgps = find(gps(:,2)==n+16,1);
        if ~isempty(kgps)
            dpos = (ins.pos-pos_1)*t1/T;
            posS = gps(kgps,3:5)' + dpos; %M5*ins.vn*t1;
            kf.Hk(:,16:19) = [-ins.MpvCnb, -ins.Mpvvn];
            kf.Rk = diag([gps(kgps,8:9)/glv.Re,gps(kgps,10)]*1)^2;
            kf = kfupdate(kf, ins.pos-posS, 'M');
            ins.qnb = qdelphi(ins.qnb, kf.xk(1:3));
            ins.vn = ins.vn - kf.xk(4:6);   ins.pos = ins.pos - kf.xk(7:9);
            ins.eb = ins.eb + kf.xk(10:12); ins.db = ins.db + kf.xk(13:15); kf.xk(1:15) = 0;
        end
    end
    pl = ins.pos + ins.MpvCnb*kf.xk(16:18) + ins.Mpvvn*kf.xk(19);
    res(kk,:) = [[ins.att;ins.vn;pl]; imu(knn,7)]';
    err(kk,:) = [kf.xk; diag(kf.Pxk); imu(knn,7)]';
    kk = timebar;
end
navplot(res, nts)
naverrplot(err(:,[1:kf.n,end]))
naverrplot(sqrt(err(:,[kf.n+[1:kf.n],end])));

