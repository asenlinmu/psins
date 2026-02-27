function avp = sinsEx(imu, att0, pos0, vn0, ts)
    if nargin==4  % avp = sinsEx(imu, att0, pos0, ts)
        ts = vn0;
        vn0 = zeros(3,1);
    end
    if nargin==3  % avp = sinsEx(imu, avp0, ts)
    	ts = pos0;
    	vn0 = att0(4:6); pos0 = att0(7:9); att0 = att0(1:3);
    end
    ss = sins([att0; vn0; pos0], ts); 
    threshold = 1000;
    height0 = pos(3)-threshold; height1 = pos(3)+threshold;
    nn = 4;
    len = fix(length(imu)/nn);    avp = zeros(len, 10);
    timebar(1, len);
    for k=1:len
        wvm = imu((k-1)*nn+1:k*nn, :);
        ss = sins(ss, wvm);          
        % ss.pos(3) = pos0(3);  
        if ss.pos(3) > height1,   ss.pos(3) = height0;
        elseif ss.pos(3)<height0, ss.pos(3) = height1;
        end
        avp(k,:) = [ss.avp; k*nn*ts]';
        timebar;
    end
%     navplot(avp, nn*ts);