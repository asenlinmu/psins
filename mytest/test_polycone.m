
afa = 10*glv.deg; omega = 2*2*pi; hh = 0.1; n = 10; th = hh/n;
% [thetap, thetac, q0] = polycone(afa, omega, hh, n, 6);
% figure(3), plot([thetap- thetac]); grid on

% kk = 1000;
% [thetap, thetac, q0] = polycone(afa, omega, hh, n*kk, 6);
% qr = q0; q0k = zeros(length(thetap),4);
% for k=1:length(thetap)
%     qr = qmul(qr,rv2q(thetap(k,:)'));
%     q0k(k,:) = qr';
% end
% q0k = q0k(kk:kk:end,:);
% thetap = sumn(thetap, kk); 
% thetac = sumn(thetac, kk); 

    t = 0;
    nn = 4; res1=zeros(length(thetap)/nn, 3); res2 = res1;
    q1 = q0; q2 = q0;  kk = 1;
    for k=1:nn:length(thetap)
        t = t+nn*th;
        wm = thetac(k:k+nn-1, :);
        qr = [cos(afa/2); sin(afa/2)*cos(omega*t); sin(afa/2)*sin(omega*t); 0]; % 姿态四元数真值
%         wm = thetap(k:k+nn-1, :);
%         qr = q0k(k+nn-1,:)';
        phi1 = cnscl(wm);               phi2 = subsamples(wm, 2000./3000);
        q1 = qmul(q1,rv2q(phi1));       q2 = qmul(q2,rv2q(phi2));
        res1(kk,:) = qq2phi(q1,qr)';    res2(kk,:) = qq2phi(q2,qr)';
        kk = kk + 1;
    end
    figure(1), plot([res1(:,1:3),res2(:,1:3)]/glv.sec), grid on
    
