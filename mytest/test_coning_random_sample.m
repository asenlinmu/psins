glvs
afa = 1.0*glv.min; omega = 2*pi*(10.0/1);
Tm = 0.030; nn = 3; T = 60;
a=0.005; b = 0.015; hh = a + (b-a).*rand(fix(T/(Tm/nn)),1);  % plot(t), grid on
[wm, qr, t] = coneSimulatorRandom1(afa, omega, hh); q0 = qr(1,:)'; qr(1,:) = [];
[wm1, qr1] = coneSimulator(afa, omega, Tm/nn, t(end)*1.5); q01 = qr1(1,:)'; qr1(1,:) = [];
qz = 1*glv.sec; wm = quantiz(wm,qz)*qz; wm1 = quantiz(wm1,qz)*qz; 
len = fix(length(wm)/nn); res = zeros(len, 6);
q1 = q0'; q2 = q01';  errT = [0;0];
for k=1:len
	wmi = wm((k-1)*nn+1:k*nn, :); 
    hi=t((k-1)*nn+2:k*nn+1)-t((k-1)*nn+1:k*nn); 
    [coef,err] = coneCoef(hi,afa,omega); errT = errT+err*sum(hi);
	q0 = qr(k*nn,:)'; q01 = qr1(k*nn,:)';
	phim1 = (sum(wmi)+cross(coef(1)*wmi(1,:)+coef(2)*wmi(2,:),wmi(3,:)))';
    q1 = qmul(q1,rv2q(phim1)); % 圆锥优化算法
    wmi1 = wm1((k-1)*nn+1:k*nn, :);
    phim2 = cnscl(wmi1);    q2 = qmul(q2,rv2q(phim2)); % 多项式近似算法
	res(k,:) = [qq2phi(q1,q0); qq2phi(q2,q01)]';
end
tt = (1:len)'*Tm;
epsilon = coneDrift(afa, omega, Tm/nn, nn); % 理论圆锥误差
figure(3)
subplot(221), plot(tt,res(:,[1,4])/glv.sec), ylabel('\it\phi_x\rm / \prime\prime'); grid on 
subplot(222), plot(tt,res(:,[2,5])/glv.sec), ylabel('\it\phi_y\rm / \prime\prime'); grid on
subplot(223), plot(tt,[res(:,3),0*tt*abs(err(1)),0*tt*abs(err(2))]/glv.sec), ylabel('\it\phi_z\rm / \prime\prime'); grid on
xlabel('\itt \rm/ s');
subplot(224), plot(tt,[res(:,6),0*tt*epsilon]/glv.sec), ylabel('\it\phi_z\rm / \prime\prime'); grid on
xlabel('\itt \rm/ s');
return
