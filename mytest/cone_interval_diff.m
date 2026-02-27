clear all
glvs
afa = 1*glv.deg; omega = 2*pi*10;
th = (0.02:(-0.01/100):0.01)';  % th = 0.01*ones(600,1);
th = (mod([1:600]',2)+1)*0.005;
t = cumsum(th); t = [0; t(1:end-1)];

wm = [ -2*sin(afa)*sin(omega*th/2).*sin(omega*(t+th/2)), ...  % 角增量
        2*sin(afa)*sin(omega*th/2).*cos(omega*(t+th/2)), ...
       -2*omega*th*sin(afa/2)^2.*ones(size(t)) ];
wm(end,:) = [];
qt = [cos(afa/2)*ones(size(t)), ...
      sin(afa/2)*cos(omega*t), ...
      sin(afa/2)*sin(omega*t), ...
      zeros(size(t))]; % 姿态四元数真值
q0 = qt(1,:)';  nn = 2;
len = fix(length(wm)/nn); res = zeros(len, 4);
for k = 1:len
    wmi = wm((k-1)*nn+1:k*nn,:); hh = th((k-1)*nn+1:k*nn);
    [kk, phim] = coneCoef(hh, wmi);
    phim = cnscl(wmi);
    q0 = qmul(q0, rv2q(phim));
    qr = qt(k*nn+1,:);
    res(k,:) = [qq2phi(q0, qr); t(k*2)]';
end
figure,plot(res(:,4),res(:,1:3)/glv.sec); grid on
    
return
% LS coefficient
X = coneCoef([1 1 ])
kk = 1; idx = 0.1:0.1:10;
for k=idx
    x(kk) = coneCoef([1 k]); kk = kk+1;
end
plot(idx, x/(2/3)), grid on

