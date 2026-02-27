function [imu, eb] = imudeldrift(imu, t0, t1, avp)
% For MEMS-grade IMU and using static-base condition within time interval
% [t0,t1], correct the gyro output by deleting their bias.
%
% Prototype: imu = imudeldrift(imu, t0, t1)
% Inputs: imu - raw SIMU data
%         t0,t1 - assuming a static-base condition time interval
% Output: imu - new SIMU date with gyro bias deleted
%
% See also  imuadderr, imurepair, imuresample.

% Copyright(c) 2009-2017, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 25/06/2017, 30/11/2020
    ts = diff(imu(1:2,end));
    if nargin>3
        if size(avp,2)>1,  avp = getat(avp,t0);   end
        wbib = imustatic(avp, 1, 1);
        wbib = wbib(1:3)*ts;
    else
        wbib = zeros(3,1);
    end
    if nargin<3, t1=t0+10; end
    idx0 = find(imu(:,end)>t0,1);
    idx1 = find(imu(:,end)>t1,1);
    eb = mean(imu(idx0:idx1,1:3),1)-wbib;
    imu(:,1:3) = [imu(:,1)-eb(1), imu(:,2)-eb(2), imu(:,3)-eb(3)];
    eb = eb/diff(imu(1:2,end));
    

