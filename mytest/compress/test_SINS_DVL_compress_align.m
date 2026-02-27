% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/02/2014
glvs
[nn, ts, nts] = nnts(4, 0.01);
kftypedef('test_SINS_DVL_19');
% [data, imu, avp0, gps, od] = zjh_read('π„÷›∫£ ‘\S14(2).dat');  vb=scale2vect(gps(:,5)); imuplot(imu)
% [att0, res] = aligni0vb(imu(1:500/ts,:), vb, avp0(1,7:9), ts); 
% lenn=500/ts; cn=100; imu1 = imucompress(imu(1:lenn,:), cn); vb1=vb(cn:cn:end,:); imuplot(imu1);
imuerr = imuseterr([0.01; 0.001; 100; 10]);
avp00 = [res.att0', qmulv(a2qua(res.att0'),vb(1,:)')', avp0(1,7:9)]';
davp0 = avpseterr([30;-30;-30]*6.0, [1;1;1], [1;1;3]*10);
lever = [1; 2; 3]*0; dT = 0.1*0;
ins = insinit(avpadderr(avp00,davp0*0), cn*ts); 
% ins = insinit([res.att0', qmulv(a2qua(res.att0'),vb(1,:)')', avp0(1,7:9)]', cn*ts); 
%% kf
r0 = [1; 1; 1/10]*1.0;
kf = kfinit(cn*ts, davp0, imuerr, lever, dT, r0);
len = length(imu1); [avp, xkpk] = prealloc(fix(len), 10, 2*kf.n+1);
timebar(1, len, '19-state SINS/DVL simulation.'); ki = 1;
for k=1:len
    wvm = imu1(k,1:6); t = imu1(k,end);
    ins = insupdate_compress(ins, wvm); 
    kf.Phikk_1 = kffk(ins, kf.fkno);
    kf = kfupdate(kf);
    vr = ins.Cnb*vb(k,:)';
	kf = kfupdate(kf, ins.vn-vr, 'M');
    avp(ki,:) = [ins.avp', t];
    xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';  ki = ki+1;
    timebar;
end
avp(ki:end,:) = []; xkpk(ki:end,:) = [];
avperr = avpcmp(avp, avp0);
% insplot(avp);
% kfplot(xkpk, avperr, imuerr, lever, dT);
%%  kf1
ins1 = ins; ins1.ts = ts;
kf1 = kfinit(nts, davp0, imuerr, lever, dT, r0); kf1.xk = kf.xk; kf1.Pxk = kf.Pxk;
len = length(imu); [avp1, xkpk1] = prealloc(fix(len/nn), 10, 2*kf.n+1);
timebar(nn, len-500/ts, '19-state SINS/DVL simulation.'); ki = 1;
for k=500/ts+1:nn:len-nn+1
    k1 = k+nn-1; 
    wvm = imu(k:k1,1:6); t = imu(k1,end);
    ins1 = insupdate(ins1, wvm);
    kf1.Phikk_1 = kffk(ins1, kf1.fkno);
    kf1 = kfupdate(kf1);
    if mod(k1,100)<nn
        vr = ins1.Cnb*vb(k,:)';
        kf1 = kfupdate(kf1, ins1.vn-vr, 'M');
        [kf1, ins1] = kffeedback(kf1, ins1, 1, 'H');
        avp1(ki,:) = [ins1.avp', t];
        xkpk1(ki,:) = [kf1.xk; diag(kf1.Pxk); t]';  ki = ki+1;
    end
    timebar;
end
avp1(ki:end,:) = []; xkpk1(ki:end,:) = [];
avp = [avp; avp1]; xkpk = [xkpk; xkpk1];
avperr = avpcmp(avp, avp0);
insplot(avp);
kfplot(xkpk, avperr, imuerr, lever, dT);

