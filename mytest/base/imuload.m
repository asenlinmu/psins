function imu = imuload(fname, description)
% There may exist different type of IMU log files, code and call
% this function to read specific IMU log file.
%
% See also  imuplot, gpsplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/03/2014
    switch(description)
        case 'leador',
            imu = imuloadleador(fname);
        case 'WangJun',
            imu = imuloadWangJun(fname);
    end
        
function gps = imuloadleador(fname)
global glv

function gps = imuloadWangJun(fname)
global glv
