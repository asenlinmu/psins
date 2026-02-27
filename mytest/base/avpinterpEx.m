function [t1, avp1] = avpinterp(t0, avp0, t, imu)
    if t0(1)>t0(2)
        t0 = flipud(t0); avp0 = flipud(avp0);
    end
    [junk, iavp, iimu] = intersect(t0, t);
    t1 = t(iimu(1):iimu(end));
    avp1 = zeros(length(t1), 9);
    kk = 1;
    timebar(1, length(iimu));
    for k=1:length(iimu)-1
        avp1(kk,:) = avp0(iavp(k),:);
        qnb = a2qnb(avp1(kk,1:3)'); vn = avp1(kk,4:6)'; pos = avp1(kk,7:9)';
        kk = kk+1;
        if mod(k,100)==1
            eth = earth(pos, vn);
        end
        for m=iimu(k)+1:iimu(k+1)-1
            ts = t(m)-t(m-1);
            vn = vn + qmulv(qnb,imu(m,4:6)') + eth.gn*ts;
            pos = pos + [vn(2)/eth.RMh; vn(1)/eth.clRNh; vn(3)]*ts;
            qnb = qmul(qnb, rv2q(imu(m,1:3)'));
            avp1(kk,:) = [q2att(qnb); vn; pos]';
            kk = kk+1;
        end
        timebar;
    end
    avp1(end,:) = avp0(iavp(end),:);
        
    