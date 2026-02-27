glvs
kftypedef('leador_POS_fusion_34');
%% init
ts = 0.005; nn = 10;  nts = nn*ts;
% imu = bfile('imu0204.bin',7); imu=imu(:,[2:7,1]); imuplot(imu(1:200:end,1:6))
% gps = bfile('gps0204.bin',15); gpsplot([gps(:,[7:9,3:5]),gps(:,6)==1,gps(:,2)])
% ta = [600,1000]/ts; att0 = aligni0(imu(ta(1):ta(2),1:6), gps(1,3:5)', ts);
%% forward
ins = insinit([att0',[0,0,0],gps(1,3:5)]', ts);
kf = kfinit(nts);
T = 3600;  14500; % imu0204:13400s
gpsPos = [gps(:,3:5),gps(:,6)>0,gps(:,2)-16];
ps = POSProcessing(kf, ins, imu(ta(2)+(2:T/ts),:), gpsPos, 'avped');
% insplot(ps.avp, 'avp');  navplot(ps.iavp, 'avp');
% kfplot(ps.xkpk);
% kfplot(ps.ixkpk);
%% fusion
psf = POSFusion(ps.avp, ps.xkpk, ps.iavp, ps.ixkpk);
xr = psf.rf(:,[1:9,end]);
psf.xf = avpcmp(psf.rf,xr); 
psf.x1 = avpcmp(psf.r1,xr); 
psf.x2 = avpcmp(psf.r2,xr);
% insplot(psf.rf(:,[1:9,end]), 'avp');
% [sltpos, e1] = sltgplot([rf(:,1:end-1),rf(:,end)], pf, gpsPos);
% POSplot(psf);
%% save
return;
save([glv.datapath,'leadorposres0204.mat'], 'ps', 'gpsPos');
load leadorposres0204.mat