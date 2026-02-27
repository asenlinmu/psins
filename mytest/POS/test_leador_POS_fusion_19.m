glvs
kftypedef('leador_POS_fusion_19');
%% init
ts = 0.005; nn = 10;  nts = nn*ts;
% imu = bfile('imu0204.bin',7); imu=imu(:,[2:7,1]); imuplot(imu(1:200:end,1:6))
% gps = bfile('gps0204.bin',15); gpsplot(gps(:,[7:9,3:5]),gps(:,2),gps(:,6)>0)
% kalign = [600,1000]/ts; [att0,res] = align_i0(imu(kalign(1):kalign(2),1:6), gps(1,3:5)', ts);
%% forward
ss = sins([att0',[0,0,0],gps(1,3:5)], ts);
kf = kf_myinit(nts);
T = 1340;  14500; % imu0204:13400s
gpsPos = [gps(:,3:5),gps(:,2)-16,gps(:,6)>0];
[avp, xkpk, iavp, ixkpk] = ...
    POSProcessing(kf, ss, imu(kalign(2)+(1:T/ts),:), gpsPos, 'avp', 'avp');
% navplot(avp(:,[1:9,end]));
% kf_plot(xkpk, avp(:,end));
% navplot(iavp(:,[1:9,end]));
% kf_plot(ixkpk, iavp(:,end));
%% fusion
[rf, pf, r1, p1, r2, p2] = POSFusion(avp, xkpk, iavp, ixkpk);
xf = avpcmp(rf,rf(:,[1:9,end]),zeros(1,10));
x1 = avpcmp(r1,rf(:,[1:9,end]),zeros(1,10));
x2 = avpcmp(r2,rf(:,[1:9,end]),zeros(1,10));
% navplot(rf(:,[1:9]),1);
% sltpos = sltgplot([r2(:,1:end-1),r2(:,end)], gpsPos);
% fusionplot(xf, x1, x2, xf(:,end));
% fusionplot(sqrt(pf), sqrt(p1), sqrt(p2), xf(:,end));
%% save
return; 
save([glv.datapath,'leadorposres0204.mat'], ...
    'rf', 'pf', 'r1', 'p1', 'r2', 'p2', 'avp', 'xkpk', 'iavp', 'ixkpk', 'gpsPos');
load leadorposres0204.mat