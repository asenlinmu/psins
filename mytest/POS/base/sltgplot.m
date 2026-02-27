function [sltpos, errt] = sltgplot(rf, posGPS, sigValid)
global glv
    if ~exist('sigValid', 'var'),  
        sigValid = ones(size(posGPS(:,end)));  
    end
    if size(posGPS,2)==5, 
        sigValid=posGPS(:,5); 
    end
    pos = rf(:,7:9);
    %eth = earth(rf(1,7:9)');
    for k=1:length(rf)
        secl = sec(rf(k,7)); f_Rh = 1/(glv.Re+rf(k,9));
        M5 = [0, f_Rh, 0; secl*f_Rh, 0, 0; 0, 0, 1];
        M5C = M5*a2mat(rf(k,1:3)');
        pos(k,:) = (rf(k,7:9)' + M5C*rf(k,16:18)' + M5*rf(k,4:6)'*rf(k,19))';
    end
    gpsplot([posGPS(:,1:3),posGPS], 0, sigValid);
    subplot(223), hold on
    plot(rf(:,end), [[pos(:,1)-posGPS(1,1),(pos(:,2)-posGPS(1,2))*cos(pos(1,1))]*glv.Re,pos(:,3)-posGPS(1,3)], 'm:'); 
    [t,is,ig] = intersect(rf(:,end), posGPS(:,4));
    subplot(221), hold off, 
    err = [[posGPS(ig,1)-pos(is,1),(posGPS(ig,2)-pos(is,2))*cos(pos(1,1))]*glv.Re,posGPS(ig,3)-pos(is,3)];
    plot(t, err);  xygo('dP');
    sltpos = pos; errt = [err, t];