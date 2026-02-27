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
imu0=imu(4780:end,:);
gps0=gps(:,[3:5,2]); gps0(:,4)=gps0(:,4)-16;
[i1, i2] = datacut(imu0(:,end), gps0(:,4));
imu0=imu0(i1,:); gps0=gps0(i2,:);
[i2, i1] = datacut(gps0(:,4), imu0(:,end));
imu0=imu0(i1,:); gps0=gps0(i2,:);

poss = gps(:,[3:5,2]);
imufile('wjimu', imu0(i1,:), [zeros(1,6),poss(1,1:3),-1]', 0.005, [1,200]);
[imu1, avp0, ts] = imufile('wjimu');