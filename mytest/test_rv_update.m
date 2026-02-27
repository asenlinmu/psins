% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/05/2014
glvs
[nn, ts, nts] = nnts(2, 0.005);
% load('D:\jiangwei\imu_raw.mat'); 
imu = [IMU_raw(:,5:7),IMU_raw(:,2:4),IMU_raw(:,1)+16]; 
imu = rfu(imu,'frd'); imu(:,6)=imu(:,6)+9.8*ts;
%% init
qnb = [1; 0; 0; 0];
rv = [0; 0; 0];
len = length(imu)/16; res = prealloc(fix(len/nn), 7);
timebar(nn, len, '19-state SINS/GPS simulation.'); ki = 1;
for k=1:nn:len-nn+1
    k1 = k+nn-1; 
    wvm = imu(k:k1,1:6); t = imu(k1,end);
    [phim, dvbm] = cnscl0(wvm);
    qnb = qmul(qnb, rv2q(phim));
    rv1 = rv+phim/2;  n2=rv1'*rv1; n=sqrt(n2);
    rp = cros(rv1,phim);
    rv = rv + phim + 1/2*rp + 1/n2*(1-n/2*cot(n/2))*cros(rv1,rp);
    res(ki,:) = [q2att(qnb); m2att(rv2m(rv)); t]'; ki = ki+1;
    timebar;
end
res(ki:end,:) = [];
figure, 
subplot(311), plot(res(:,7)-res(1,7), [res(:,1),res(:,4)]/glv.deg), grid on
subplot(312), plot(res(:,7)-res(1,7), [res(:,2),res(:,5)]/glv.deg), grid on
subplot(313), plot(res(:,7)-res(1,7), [res(:,3),res(:,6)]/glv.deg), grid on
