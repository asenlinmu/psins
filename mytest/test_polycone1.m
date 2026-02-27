function test_polycone1
global glv
afa = 10*glv.deg; omega = 4*2*pi; th = 0.01; T=6;
[wm, qr] = coneSimulator(afa, omega, th, T);
nn = 2;
len = fix(length(wm)/nn);
res = zeros(len, 6);
q1 = qr(1,:)'; q2 = q1;
for k=1:len
	wmi = wm((k-1)*nn+1:k*nn, :);
	q0 = qr(k*nn+1,:)';
	phim1 = cnscl(wmi);            q1 = qmul(q1,rv2q(phim1)); % 圆锥优化算法
    phim2 = cnscl(wmi, 'poly');    
    phim2 = comps(phim2,wmi); 
    q2 = qmul(q2,rv2q(phim2)); % 多项式近似算法
	res(k,:) = [qq2phi(q1,q0); qq2phi(q2,q0)]';
end
tt = (1:len)'*nn*th;
figure(1)
subplot(311), plot(tt,res(:,[1,4])/glv.sec), grid on
subplot(312), plot(tt,res(:,[2,5])/glv.sec), grid on
subplot(313), plot(tt,res(:,[3,6])/glv.sec), grid on

function phim = comps(phim, wm)
    n = size(wm,1);
    if n==2
        c = cross(wm(1,:),wm(2,:));
         dphim = -10/60*cross((1*wm(1,:)-1*wm(2,:)),c);
%         dphim = -1/60*cross((7*wm(1,:)+3*wm(2,:)),c);
%         dphim = 1/60*cross((15*wm(1,:)-5*wm(2,:)),c);
    elseif n==4
        w1 = wm(1,:) + wm(2,:);
        w2 = wm(3,:) + wm(4,:);
        dphim = 1/60*cross((7*w1+3*w2),cross(w1,w2));
    end
    phim = phim+dphim';
    
 return;   
 theta1 = randn(3,1); theta2 = randn(3,1);
 ha = 3*theta1-theta2; h2b = -2*theta1+2*theta2;
 a = 1/48*askew(ha)^2*h2b-1/60*askew(h2b)^2*ha;
 b = 1/60*cross(7*theta1+3*theta2, cross(theta1,theta2));
 [a- b]