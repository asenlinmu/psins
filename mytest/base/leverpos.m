function [pos, pp] = leverpos(avp, x, p)
global glv
    pos = avp(:,7:9); pp = pos;
    for k=1:length(avp)
        avp0 = avp(k,:);
        secl = sec(avp0(7)); f_Rh = 1/(glv.Re+avp0(9));
        M5 = [0, f_Rh, 0; secl*f_Rh, 0, 0; 0, 0, 1];
        M5C = M5*a2cnb(avp0(1:3)');
        pos(k,:) = (avp0(7:9)' - x(k,7:9)' + M5C*x(k,16:18)')';
        pp(k,:) = p(k,7:9) + diag(M5C*diag(p(k,16:18))*M5C')';
    end