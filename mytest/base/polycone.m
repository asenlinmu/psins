function [thetapoly, thetacone, q0] = polycone(afa, omega, hh, n, T)
% use ploynomial to fit coning motion
    t = (0:hh:T)'; 
    w = [ -omega*sin(afa)*sin(omega*t), ...  % angular rate
           omega*sin(afa)*cos(omega*t), ...
          -2*omega*sin(afa/2)^2*ones(size(t)) ];
    px = spline(t, w(:,1));  py = spline(t, w(:,2));  pz = spline(t, w(:,3));
    thetapoly = zeros(length(t)*n-n,3);
    th = hh/n;  thi = (0:th:hh)';   coef = [1/4, 1/3, 1/2, 1];
    kk = 1;
    for k=1:px.pieces  % angular increment
        pxk = [coef.*px.coefs(k,:), 0];  thetax = polyval(pxk, thi);
        pyk = [coef.*py.coefs(k,:), 0];  thetay = polyval(pyk, thi);
        pzk = [coef.*pz.coefs(k,:), 0];  thetaz = polyval(pzk, thi);
        thetapoly(kk:kk+n-1, :) = diff([thetax, thetay, thetaz]);
        kk = kk + n;
    end
    %%
    t = (0:th:T-th)';
    thetacone = [ -2*sin(afa)*sin(omega*th/2)*sin(omega*(t+th/2)), ...
                   2*sin(afa)*sin(omega*th/2)*cos(omega*(t+th/2)), ...
                  -2*omega*th*sin(afa/2)^2*ones(size(t)) ];
    q0 = [cos(afa/2); sin(afa/2)*cos(omega*0); sin(afa/2)*sin(omega*0); 0];
