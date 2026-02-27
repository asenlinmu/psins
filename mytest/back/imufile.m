function [imu, pos] = imufile(fname, imu, pos, t1, ts, scales, infostr)
% Create or Read PSINS compact text-format SIMU file. The text file has
% high compress ratio.
% Inputs & Outputs: 
%    fname - text file name
%    imu - IMU incremental sampling data (gyro in rad, acc in m/s)
%    pos - IMU initial position=[lat;lon;height], (lat % lon in deg)
%    t1 - sampling time of the first IMU record
%    ts - sampling interval (in time second)
%    scales - scale factors for each IMU column (gyro in sec, acc in ug*s)
%    infostr - user's infomation string
%
% See also  gpsfile, imuresample.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/08/2011, 09/10/2013, 12/03/2014
global glv
    if isempty(strfind(fname, '.'))
        fname = [fname,'.imu']; 
    end
    if nargin>1 % write
        if nargin<7,  infostr='by Gongmin Yan';  end
        if nargin<6,  scales=0;  end
        if nargin<5,  ts=mean(diff(imu(:,7)));  end
        if nargin<4,  t1=ts;  end
        scales = scales(:)';
        if length(scales)<2,  scales=[0.5, 200];  end
        if length(scales)==2,  scales=[repmat(scales(1),1,3), repmat(scales(2),1,3)];  end
        scales0 = scales;
        scales = [scales(1:3)*glv.sec, scales(4:6)*glv.ug];
        pos = [pos(1)/glv.deg; pos(2)/glv.deg; pos(3)];
        imu = imu(:,1:6);
        for k=1:6,  imu(:,k) = imu(:,k)/scales(k);  end
        imu = diff([zeros(1,6); fix(cumsum(imu,1))]);
        fid = fopen([glv.datapath,fname], 'wt');
        fprintf(fid, '%% DO NOT EDIT THIS HEADER !!!\n');
        fprintf(fid, '%% %s %s\n', datestr(now,31), infostr);
        fprintf(fid, '%% %.8f %.8f %.3f %f %f //pos,t1,ts\n', pos(1), pos(2), pos(3), t1, ts);
        fprintf(fid, '%% %f %f %f %.3f %.3f %.3f //scale factors(sec,ug*s)\n', scales0);
        fprintf(fid, '%d %d %d %d %d %d\n', imu');
        fclose(fid);
    else % read
        fid = fopen([glv.datapath,fname], 'rt');
        t1 = fgetl(fid);
        t2 = fgetl(fid); % infostr = t2(3:end);
        t3 = fgetl(fid); t3 = str2num(t3(2:find(t3=='/')-1)); 
        pos = [t3(1:2)*glv.deg,t3(3)]'; t1 = t3(4); ts = t3(5);
        t4 = fgetl(fid); t4 = str2num(t4(2:find(t4=='/')-1));
        scales = [t4(1:3)*glv.sec, t4(4:6)*glv.ug];
        fclose(fid);
        imu = load([glv.datapath,fname]);
        for k=1:6,  imu(:,k) = imu(:,k)*scales(k);  end
        imu(:,7) = t1+(0:length(imu)-1)'*ts;
    end
