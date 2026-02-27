glvs
trj.ts = 0.01;       % sampling interval
trj.avp0 = avpset([0;0;10], 0, [34.246048;108.909664;380]);
% trajectory motion setting
x = [];
mn = trjsegment('init', 0);
mn = trjsegment(mn, 'uniform',      100);
mn = trjsegment(mn, 'accelerate',   1, x, 0.1);
mn = trjsegment(mn, 'deaccelerate', 1, x, 0.1);
mn = trjsegment(mn, 'uniform',      100);
% generate, save & plot
[trj.imu, trj.avp] = trjsimu(trj.avp0, mn.wat, trj.ts, 2);
insplot(trj.avp);
imuplot(trj.imu);

imuerr = imuseterr([0.01;0.001;50;5]);
imu = imuadderr(trj.imu, imuerr);
[atti0, resi0] = aligni0(imu, trj.avp0(7:9), trj.ts);
resfit = aligni0fit(resi0, resi0.lat, resi0.nts);
