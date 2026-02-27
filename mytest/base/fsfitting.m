function [a0, anbn, w1] = fsfitting(tx, N)
% FS fitting.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/06/2014
    if size(tx,2)==1
        tx = [(0:length(tx)-1)', tx];
    end
	t = tx(:,1); x = tx(:,2);  % time series
    q = (x(2:end)-x(1:end-1))./(t(2:end)-t(1:end-1)); % linear fitting
    p = x(2:end)-q.*t(2:end);    % x = p + q*t
    lenp = length(p);
    T1 = t(end)-t(1); w1 = 2*pi/T1; % time span % fundamental frequency
    a0 = 0;  % DC component
    for k=1:lenp
        a0 = a0 + p(k)*(t(k+1)-t(k))+q(k)/2*(t(k+1)^2-t(k)^2);
    end
    a0 = a0/T1;
	anbn = zeros(N,2);  % harmonic components
    for n=1:N
        nw1 = n*w1; nw12 = nw1*nw1;
        t1 = t(2:end); t0 = t(1:end-1);
        s1 = sin(nw1*t1); s0 = sin(nw1*t0);
        c1 = cos(nw1*t1); c0 = cos(nw1*t0);
        s = s1-s0; c = c1-c0;
        ts = t1.*s1-t0.*s0; tc = t1.*c1-t0.*c0;
        a =  p/nw1.*s + q.*(1/nw12*c+1/nw1*ts);
        b = -p/nw1.*c + q.*(1/nw12*s-1/nw1*tc);
        anbn(n,:) = sum([a,b],1)*2/T1;
    end
