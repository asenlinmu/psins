function imu = imucompress(imu0, n)
% Compress high-frequency IMU sampling data to slow-frequency data.
%
% Prototype: imu = imucompress(imu0, n)
% Inputs: imu0 - high-frequency IMU sampling data
%         n - compress ratio
% Output: imu - low-frequency IMU sampling data, including coning &
%               sculling compensation
%
% See also  imuresample, insupdate_compress.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/08/2014
    nn = 2;
    if mod(n,nn)~=0, error('Compress ratio is not correct.'); end
    len = length(imu0); imu = zeros(fix(len/n), 7);
    qib = [1;0;0;0]; vi = [0;0;0];
    timebar(n, len, 'IMU data compress.'); ki = 1;
    for k=1:nn:len-nn+1
        k1 = k+nn-1;
        [phim,dvbm] = cnscl(imu0(k:k1,1:6)); t = imu0(k1,7);
        vi = vi + qmulv(qib, dvbm);
        qib = qupdt(qib, phim);
        if mod(k1,n) == 0
            imu(ki,:) = [q2rv(qib); vi; t]'; ki = ki + 1;
            qib = [1;0;0;0]; vi = [0;0;0];
            timebar;
        end
    end
    imu(ki:end,:) = [];