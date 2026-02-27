% SINS/GPS intergrated navigation simulation unsing kalman filter, with
% 19-state included:
%       phi(3), dvn(3), dpos(3), eb(3), db(3), lever(3), dT(1)
% Please run 'test_SINS_trj.m' to generate 'trj10ms.mat' beforehand!!!
% See also  test_SINS_GPS.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/05/2014
glvs
kftypedef('test_SINS_GPS_19');
[nn, ts, nts] = nnts(2, 0.005);
load('D:\jiangwei\imu_raw.mat'); load('D:\jiangwei\gps'); load('D:\jiangwei\ref'); 
imu = [IMU_raw(:,5:7)*ts,IMU_raw(:,2:4)*ts,IMU_raw(:,1)+16]; imu = rfu(imu,'frd'); imu(:,6)=imu(:,6)+9.8*ts;
gps = GPS(:,[5:7,2:4,1]); gps(:,1:3) = rfu(gps(:,1:3),'frd');
i0=imu(1,7); imu(:,7)=imu(:,7)-i0; gps(:,7)=gps(:,7)/1000-i0; [imu(1,7),gps(1,7)]
ref0 = [ref(:,[2,1,3,7:9,4:6]),gps(:,end)]; ref0(:,3) = yawcvt(ref0(:,3),'c360cc180'); ref0(:,4:6) = rfu(ref0(:,4:6),'frd');
% imu(1:13.70/ts,:) = []; 
% ref0(:,1:2)=ref0(:,1:2); imu(:,4:5) = imu(:,4:5)/2;
% imuplot(imu); gpsplot(gps); insplot(ref0);
%% init
kstart = 0*200+1;
imuerr = imuseterr([30; 01; 10000; 100]);
davp0 = avpseterr([300*60;300*60;300], [1;1;1], [1;1;1]);
lever = [1; 2; 3]*0; dT = 0.0;
att0 = ref0((kstart-1)/200+1,1:3); att0(1:2) = att0(1:2)+[1,-1]*glv.deg;
avp0 = [att0,gps((kstart-1)/200+1,1:6)]';
avp0 = [[-1.729593 1.448135 360-178.136292]*glv.deg  0.101 -3.653 -0.072...
    -0.590996259443543   2.639295592916946  84.474236710928380]';
imugpssyn(imu(:,7), gps(:,7));
ins = insinit(avp0, ts); 
%% kf
kf = kfinit(nts, davp0, imuerr, lever, dT);
len = length(imu)/8; [avp, xkpk, res0, res1] = prealloc(fix(len/nn), 10, 2*kf.n+1, 10, 10);
timebar(nn, len, '19-state SINS/GPS simulation.'); ki = 1;
for k=kstart:nn:len-nn+1
    k1 = k+nn-1; 
    wvm = imu(k:k1,1:6); t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins, kf.fkno);
    kf = kfupdate(kf);
    [kgps, dt] = imugpssyn(k, k1, 'F');
    if kgps>0
        posGPS = gps(kgps,4:6)';
        kf.Hk(:,16:19) = [-ins.MpvCnb, -ins.Mpvvn];
        kf = kfupdate(kf, ins.pos-ins.Mpvvn*dt-posGPS, 'M');
        [kf, ins] = kffeedback(kf, ins, 1, 'V');
        res0(ki,:) = [ins.avp; t]';
        res1(ki,:) = [ref0(kgps,1:9)'; t]';
        avp(ki,:) = [ins.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';  ki = ki+1;
    end
    timebar;
end
avp(ki:end,:) = []; xkpk(ki:end,:) = []; res0(ki:end,:) = []; res1(ki:end,:) = [];
% insplot(avp);
% kfplot(xkpk, avperr, imuerr, lever, dT);
% inserrplot(avpcmp(res1,res0));
figure, 
subplot(221), plot(res0(:,10), [res0(:,1:2) res1(:,1:2)]/glv.deg), grid on
subplot(222), plot(res0(:,10), [res0(:,3) res1(:,3)]/glv.deg), grid on
subplot(223), plot(res0(:,10), [res0(:,4:5), res1(:,4:5)]), grid on
subplot(224), plot(res0(:,10), [res0(:,6), res1(:,6)]), grid on
figure, 
subplot(311), plot(diff([res0(:,4), res1(:,4)])), grid on
subplot(312), plot(diff([res0(:,5), res1(:,5)])), grid on
subplot(313), plot(diff([res0(:,6), res1(:,6)])), grid on