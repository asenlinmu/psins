function avpout = POSimu2gps(avpltdv, imu, vbias)
% For POS, transfer IMU AVP outputs to the point at GPS, where lever arm
% and time delay are both considered.
%
% See also inslever, insextrap.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/02/2015
global glv
    ts = diff(imu(1:2,end));
    [t, i1, i2] = intersect(avpltdv(:,end), imu(:,end));
    avpltdv = avpltdv(i1,:);  imu = imu(i2,:);
    len = length(t);
    avpout = zeros(len,9);
    if ~exist('vbias', 'var'), vbias = 0; end
    for k=1:len
        vn = avpltdv(k,4:6)'; pos = avpltdv(k,7:9)';
        secl = sec(pos(1)); f_Rh = 1/(glv.Re+pos(3));
        Mpv = [0, f_Rh, 0; secl*f_Rh, 0, 0; 0, 0, 1];
        Cnb = a2mat(avpltdv(k,1:3)');
        if imu(k,end)>3225
            a = 1;
        end
        web = imu(k,1:3)'/ts; an = Cnb*imu(k,4:6)'/ts+[0;0;-9.8];
        lever = avpltdv(k,16:18)'; dt = avpltdv(k,19); dv = avpltdv(k,35:37)';
        avpout(k,:) = [ m2att(mupdt(Cnb, web*dt*0));    % gyro may not delay
                        vn+Cnb*cros(web,lever)+an*dt+dv*vbias;
                        pos+Mpv*(Cnb*lever+vn*dt) ]';
    end
    avpout(:,10) = t;
 