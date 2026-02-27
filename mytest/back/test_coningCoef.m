glvs
afa = 10*glv.min; omega = 2*pi*10;
hh = [1.772;2;1]; %hh = [1;1;1;1];
Tm = 0.030; hh = hh/sum(hh)*Tm; nn = length(hh); T = 60;
[coef, err] = coningCoef(hh, afa, omega);
[wm, qr] = coneSimulatorRandom(afa, omega, hh, T); q0 = qr(1,:)'; qr(1,:) = [];
[wm1, qr1] = coneSimulator(afa, omega, Tm/nn, T); q01 = qr1(1,:)'; qr1(1,:) = [];
len = fix(length(wm)/nn); res = zeros(len, 6);
q1 = q0'; q2 = q01';
for k=1:len
	wmi = wm((k-1)*nn+1:k*nn, :);
	q0 = qr1(k*nn,:)';
	phim1 = (sum(wmi)+cross(coef(1)*wmi(1,:)+coef(2)*wmi(2,:),wmi(3,:)))';
    q1 = qmul(q1,rv2q(phim1)); % 圆锥优化算法
    wmi1 = wm1((k-1)*nn+1:k*nn, :);
    phim2 = cnscl(wmi1);    q2 = qmul(q2,rv2q(phim2)); % 多项式近似算法
	res(k,:) = [qq2phi(q1,q0); qq2phi(q2,q0)]';
end
tt = (1:len)'*Tm;
epsilon = conedrift(afa, omega, Tm/nn, nn); % 理论圆锥误差
figure(3)
subplot(221), plot(tt,res(:,[1,4])/glv.sec), ylabel('\it\phi_x\rm / arcsec'); grid on 
subplot(222), plot(tt,res(:,[2,5])/glv.sec), ylabel('\it\phi_y\rm / arcsec'); grid on
subplot(223), plot(tt,[res(:,3),tt*abs(err(1))]/glv.sec), ylabel('\it\phi_z\rm / arcsec'); grid on
xlabel('\itt \rm/ s');
subplot(224), plot(tt,[res(:,6),tt*err(2)]/glv.sec), ylabel('\it\phi_z\rm / arcsec'); grid on
xlabel('\itt \rm/ s');

return

afa = 1; omega = 1;
kk = 1;
a = 0.01:0.1:5; b = 0.01:0.1:5; 
% a = 1.77:0.001:1.775; b = [2,2.001]; 
errk = zeros(length(a),length(b),2);
for ka=1:length(a)
    for kb=1:length(b)
        hh = [a(ka); b(kb); 1]; hh = hh/sum(hh);
        [coef, err] = coningCoef(hh, 1, 1);
        errk(ka,kb,:) = err'; 
    end
end
hold off, surf(a,b,abs(errk(:,:,1))'), xlabel('h1/h3'), ylabel('h2/h3'); grid on
% hold on, surf(a,b,errk(:,:,2)')

return
hh = [1.772;2;1];
[coef, err1] = coningCoef(hh/sum(hh), 1, 1)
hh = [2.5;2.5]; 
[coef, err1] = coningCoef(hh, 1, 1)
k = (hh(1)+hh(2))^2/(6*hh(1)*hh(2))
e = k*1/24*(hh(1)*hh(2)*(hh(1)+hh(2))^3 + hh(1)^3*hh(2)*(hh(1)+hh(2))...
    +hh(1)*hh(2)^3*(hh(1)+hh(2))) - (hh(1)+hh(2))^5/120;
e1 = (hh(1)+hh(2))^3/24/6*(hh(1)^2+hh(2)^2-1/5*(hh(1)+hh(2))^2);
e1/2/(hh(1)+hh(2))