%% reverse
glvs
typedef = glv.wangjun_ins_gps_19;
glv.wie = -glv.wie;
if ~exist('imu', 'var');   load imu_gps;  end
ts = 0.005; nn = 4; nts = nn*ts;
kend = 301/ts; len = 200/ts;
kimu = find(imu(:,7)==attr(kend,4),1);
kgps = find(gps(:,2)==round(imu(kimu,7))+16,1);
att0 = attr(kend,1:3)'; pos0 = gps(kgps,3:5)'; vn0 = pp2vn(pos0, gps(kgps+1,3:5)');
ss = sins([att0;-vn0;pos0], ts);
imuerr = imuSetErr([0.1, 0.01, 100, 10]*10);
avpErr = [ [1;1;1]*glv.deg; [1;1;1]; [[1;1]/glv.Re;1]*100 ];
kf = kf_init(typedef, nts, avpErr, imuerr);
resi = zeros(len/nn,10); erri = zeros(len/nn,2*kf.n);
kk = timebar(nts, len*ts, 'POS reverse.');
for k=kimu:-nn:kimu-len
    knn = k-nn+1;
    wvm = imu(k:-1:knn, 1:6);  pos_1 = ss.pos;
    wvm(:,1:3) = -wvm(:,1:3);  
    ss = sins(ss, wvm);  
    Ft = etmEx(ss, typedef); M5 = Ft(7:9,4:6);
    kf.Phikk_1 = kf.I + Ft*nts;
    kf = kalman(kf);
    n = floor(imu(k,7)); t1 = imu(k,7)-n; t0 = n-imu(k-nn,7); T = t0+t1;
    if t0*t1>0 || t1==0
        kgps = find(gps(:,2)==n+16,1);
        if ~isempty(kgps)
            dpos = (ss.pos-pos_1)*t0/T;
            posS = gps(kgps,3:5)' + dpos; %M5*ss.vn*t1;
            kf.Hk(:,16:19) = [-M5*ss.Cnb, M5*ss.vn];   % dt sign change vs forward
            kf.Rk = diag([gps(kgps,8:9)/glv.Re,gps(kgps,10)]*1)^2;
            kf = kalman(kf, ss.pos-posS, 'M');
        end
    end
    [kf, ss] = kf_feedback(kf, ss, nts, glv.feedback_common_15);
%     pl = ss.pos + M5*ss.Cnb*kf.xk(16:18) + M5*ss.vn*kf.xk(19);
    pl = ss.pos + M5*ss.Cnb*[-0.253;0.011;1.738] + M5*ss.vn*(-0.001);
    resi(kk,:) = [[ss.att;ss.vn;pl]; imu(knn-1,7)]';
    erri(kk,:) = [kf.xk+kf.xtotal_fb; diag(kf.Pxk)]';
    kk = timebar;
end
naverrplot(erri(:,1:kf.n), resi(:,10)-attr(1,4));
%% forward
glv.wie = -glv.wie; ss.vn = -ss.vn;
len = 300/ts;
res = zeros(len/nn,10); err = zeros(len/nn,2*kf.n);
kk = timebar(nts, len*ts, 'POS forward.');
kimu0 = knn;
for k=kimu0:nn:kimu0+len
    knn = k+nn-1;
    wvm = imu(k:knn, 1:6);  pos_1 = ss.pos;
    ss = sins(ss, wvm);
    Ft = etmEx(ss, typedef); M5 = Ft(7:9,4:6);
    kf.Phikk_1 = kf.I + Ft*nts;
    kf = kalman(kf);
    n = floor(imu(knn,7)); t1 = imu(knn,7)-n; t0 = n-imu(knn-nn,7); T = t0+t1;
    if t0*t1>0 || t1==0
        kgps = find(gps(:,2)==n+16,1);
        if ~isempty(kgps)
            dpos = (ss.pos-pos_1)*t1/T;
            posS = gps(kgps,3:5)' + dpos; %M5*ss.vn*t1;
            kf.Hk(:,16:19) = [-M5*ss.Cnb, -M5*ss.vn];
            kf.Rk = diag([gps(kgps,8:9)/glv.Re,gps(kgps,10)]*1)^2;
            kf = kalman(kf, ss.pos-posS, 'M');
        end
    end
    [kf, ss] = kf_feedback(kf, ss, nts, glv.feedback_common_15);
%     pl = ss.pos + M5*ss.Cnb*kf.xk(16:18) + M5*ss.vn*kf.xk(19);
     pl = ss.pos + M5*ss.Cnb*[-0.253;0.011;1.738] + M5*ss.vn*(-0.001);
    res(kk,:) = [[ss.att;ss.vn;pl]; imu(knn,7)]';
    err(kk,:) = [kf.xk+kf.xtotal_fb; diag(kf.Pxk)]';
    kk = timebar;
end
naverrplot(err(:,1:kf.n), res(:,10)-attr(1,4));
return
%% compare & fusion

aef = avpcmp(res(:,[1:3,10]), attr(:,1:4), 'a');
aepf = avpcmp(res(:,[7:9,10]), [gps(:,3:5),gps(:,2)-16], 'p');
aei = avpcmp(resi(:,[1:3,10]), attr, 'a');
aepi = avpcmp(resi(:,7:10), [gps(:,3:5),gps(:,2)-16], 'p');
[a, pp] = avpfusion(res(:,[1:3,10]), err(:,kf.n+[1:3]), resi(:,[1:3,10]), erri(:,kf.n+[1:3]), 'a');
ae0 = avpcmp(a, attr, 'a');
figure,plot(ae0(:,4),ae0(:,1:3)/glv.deg,'r', aef(:,4),aef(:,1:3)/glv.deg,'g', aei(:,4),aei(:,1:3)/glv.deg,'b'), grid on
[p, pp] = avpfusion(res(:,7:10), err(:,kf.n+[7:9]), resi(:,7:10), erri(:,kf.n+[7:9]), 'p');
aep0 = avpcmp(p, [gps(:,3:5),gps(:,2)-16], 'p');
figure,plot(aep0(:,4),aep0(:,1:2)*glv.Re,'r', aepf(:,4),aepf(:,1:2)*glv.Re,'g', aepi(:,4),aepi(:,1:2)*glv.Re,'b'), grid on


%% trjInverse
kk = 5;
[trj, avp0] = trjInverse([res(:,1:3),res(:,7:10)], ts/kk, res(1,10));
trj1 = [trj(kk:kk:end,1:9), sumn(trj(1:end,10:15),kk), trj(kk:kk:end,16)];
navplot(trj1(:,1:15), ts);

plot(trj(:,end), trj(:,1:2), a(:,end), a(:,1:2)), grid on
plot(trj(:,end), trj(:,4:6), res(:,end), res(:,4:6)), grid on
plot(trj(:,end), trj(:,10:12), imu(kimu0:kimu,end), imu(kimu0:kimu,1:3)), grid on
plot(trj(:,end), trj(:,13:15), imu(kimu0:kimu,end), imu(kimu0:kimu,4:6)), grid on

%% navigation test
nn = 8; nts = nn*ts;
len = length(trj1); errr = zeros(len/nn,9); nnav = zeros(len/nn,9);
avpErr = [0;0;0; 0;0;0; [[1;1]/glv.Re;1]*0];
ss = sins(avp0, ts, avpErr);
kk = timebar(nn,len);
for k=1:nn:len-nn+1
    knn = k+nn-1;
    ss = sins(ss, trj1(k:knn,10:15));
    nnav(kk,:) = ss.avp';
    errr(kk,:) = inserr(ss, trj1(knn,1:9));
    kk = timebar;
end
navplot(nnav,nts);
naverrplot(errr,nts)

figure,plot([errr(:,[4,5]), cumsum([-errr(:,2),errr(:,1)]*glv.g0*nts)]),grid on

