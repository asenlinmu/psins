glvs
kftypedef('WangJun_POS_fusion_34');
%% init
if ~exist('imu', 'var');   load imu_gps;  end
ts = 0.005; nn = 8;  nts = nn*ts;
k0 = 200/ts; len = 3000/ts;
kimu = find(imu(:,7)==attr(k0,4), 1);
kgps = find(gps(:,2)==round(imu(kimu,7))+16, 1);
att0 = attr(k0,1:3)'; pos0 = gps(kgps,3:5)'; vn0 = pp2vn(pos0, gps(kgps+1,3:5)');
ss = sins([att0;vn0;pos0], ts);
imuerr = imuSetErr([5;0.1;5000;100], [20;0]);
avpErr = [ [1;1;1]*glv.deg; [1;1;1]; [[10;10]/glv.Re;10] ];
kf = kf_myinit(nts, avpErr, imuerr);
gpsPos = [gps(:,3:5),gps(:,2)-16];
res = POSProcessing(kf, ss, imu(kimu:kimu+len,:), gpsPos, 'avped');
%% forward
% navplot(res.avp(:,[1:9,end]));
% kf_plot(res.xkpk, res.avp(:,end));
% navplot(res.iavp(:,[1:9,end]));
% kf_plot(res.ixkpk, res.iavp(:,end));
%% fusion
xr = res.rf(:,[1:9,end]);
xf = avpcmp(res.rf,xr); x1 = avpcmp(res.r1,xr); x2 = avpcmp(res.r2,xr);
% navplot(res.rf(:,[1:9]),1);
% [sltpos, e1] = sltgplot([res.rf(:,1:end-1),res.rf(:,end)], res.pf, gpsPos);
% fusionplot(xf, x1, x2, xf(:,end));
% fusionplot(sqrt(res.pf), sqrt(res.p1), sqrt(res.p2), xf(:,end));
err = avpcmp(res.rf(:,[1:3,end]),attr); naverrplot(err,nts);
