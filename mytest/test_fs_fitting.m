Ts = 1;
t = (0:Ts:20)';
x = randn(size(t))+2;
x(1:end/2) = 0; % x(end/2:end) = 1;
x(end)=x(1);
NN = 50;
for N=1:NN
    [a0, anbn, w1] = fsfitting(x, N);
    tt = (t(1):Ts/100:t(end))';
    xx = a0;
    for n=1:N
        wt = n*w1*tt;
        xx = xx + anbn(n,1)*cos(wt) + anbn(n,2)*sin(wt);
    end
    subplot(211), plot(0:N,[a0;anbn(:,1)], 1:N,anbn(:,2),'r-*'); xlim([0,NN]); grid on
    subplot(212), plot(t,x, tt,xx, 'r'); grid on
    pause(1);
end
return;

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
figure,plot(0:N-1,x, t,real(x1), 'r-*'); grid on
