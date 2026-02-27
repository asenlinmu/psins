% Ts = 1;
% t = (0:Ts:20)';
% x = randn(size(t))+2;
% x(1:end/2) = 0; % x(end/2:end) = 1;
% x(end)=x(1);
% NN = 50;
% for N=1:NN
%     [a0, anbn, w1] = fsfitting(x, N);
%     tt = (t(1):Ts/100:t(end))';
%     xx = a0;
%     for n=1:N
%         nwt = n*w1*tt;
%         xx = xx + anbn(n,1)*cos(nwt) + anbn(n,2)*sin(nwt);
%     end
%     subplot(211), plot(0:N,[a0;anbn(:,1)], 1:N,anbn(:,2),'r-*'); xlim([0,NN]); grid on
%     subplot(212), plot(t,x, tt,xx, 'r'); grid on
%     pause(1);
% end

X = fft(x);
N = length(x);
a0 = X(1); a = real(X(2:end)); b = imag(X(2:end));
x1 = a0;
t = (0:1/50:N-1)';
for k=1:N-1
    wt = 2*pi/N*k*t;
    x1 = x1 + a(k)*cos(wt) - b(k)* sin(wt);
%     x1 = x1 + exp(1i*wt)*X(k+1);
end
x1 = x1/N;
figure,plot(0:N-1,x, t,real(x1), 'r-*')

% function test_ffff
% Ts = 1;
% t = (1:Ts:20)';
% x = randn(size(t))+2;  x(end)=x(1);
% 	q = (x(2:end)-x(1:end-1))./(t(2:end)-t(1:end-1));
%     p = x(2:end)-q.*t(2:end);
% T1 = t(end)-t(1); w1 = 2*pi/T1;
% a0 = 0;
% for k=1:length(p)
%     a0 = a0 + p(k)*(t(k+1)-t(k))+q(k)/2*(t(k+1)^2-t(k)^2);
% end
% a0 = a0/T1;
% N = 100; ab = zeros(N,2);
% for n=1:N
%     nw1 = n*w1; nw12 = nw1*nw1;
%     an = 0; bn = 0;
%     for k=1:length(p)
%         t1 = t(k+1); t0 = t(k);
%         s = sin(nw1*t1)-sin(nw1*t0);
%         c = cos(nw1*t1)-cos(nw1*t0);
%         ts = t1*sin(nw1*t1)-t0*sin(nw1*t0);
%         tc = t1*cos(nw1*t1)-t0*cos(nw1*t0);
%         an = an + p(k)/nw1*s+q(k)*(1/nw12*c+1/nw1*ts);
%         bn = bn - p(k)/nw1*c+q(k)*(1/nw12*s-1/nw1*tc);
%     end
%     ab(n,:) = [an, bn]*2/T1;
% end
% 
% tt = (t(1):Ts/100:t(end))';
% xx = a0;
% for n=1:N
%     xx = xx + ab(n,1)*cos(n*w1*tt) + ab(n,2)*sin(n*w1*tt);
% end
% plot(t,x, tt,xx); grid on
% aa=1;
%         
% 
