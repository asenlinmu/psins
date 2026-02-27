glvs
g0 = glv.g0;
h = 0;  L = 45;
clear res; ki = 1;
% for h=1:100:10000
for L=0:1:90
    sl2 = sin(L*glv.deg)^2;
    gL = g0*(1+1.93185138639e-3*sl2)/sqrt(1-6.69437999013e-3*sl2)*...
        glv.Re^2/(glv.Re+h)^2;   % Qin p213
    g = g0*(1+5.27094e-3*sl2+2.32718e-5*sl2*sl2)-3.086e-6*h;
%     res(ki,:) = [h, gL, g]; ki = ki+1;
    res(ki,:) = [L, gL, g]; ki = ki+1;
end
plot(res(:,1), [res(:,2)-res(:,3)]/g0); grid on