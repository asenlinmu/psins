% Trajectory generation for later use.
% See also  test_SINS, test_SINS_GPS, test_DR.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/06/2011, 10/02/2014
clear all
glvs
trjts = .1;       % sampling interval
trjavp0 = avpset([0;0;0], 0, [34.246048;108.909664;380]);
% trajectory motion setting
x = [];
mn = trjsegment('init', 0);
mn = trjsegment(mn, 'uniform',      10);
mn.wat = [mn.wat; [9*5, 0, 0, 0,10*glv.dps, 0, 0, 0]];
mn = trjsegment(mn, 'uniform',      10);
% generate, save & plot
[imu, trjavp] = trjsimu(trjavp0, mn.wat, trjts, 1);
% insplot(trjavp);
% imuplot(imu);
ts = trjts; nn = 1; nts = nn*ts;
ss = sins(trjavp0, ts);
len = length(imu); res = prealloc(fix(len/nn), 10);
kk = timebar(nn, len, 'SINS Simulation.');
for k=1:nn:len-nn+1
    k1 = k+nn-1;
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    if t==11
        aa=1;
    end
    ss = sins(ss, wvm);  ss.vn(1:3) = trjavp(k1,4:6);
    res(kk,:) = [ss.avp; t]';
    kk = timebar;
end
avperr = avpcmp(res, trjavp);
insplot(res);
inserrplot(avperr)
