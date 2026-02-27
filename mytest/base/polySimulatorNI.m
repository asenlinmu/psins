function [wm, qt, t] = polySimulatorNI(pp, th, T)
% polynomial motion simulation for non-uniform sampling interval
% output: wm = [          dtheta1, dtheta2, ... , dthetaN ]';
%         qt = [ q0,      q1,      q2,      ... , qN      ]';
%          t = [ t0,      t1,      t2,      ... , tN      ]';
    c = size(pp,2):-1:1; c = 1./c; 
    ppi = [[c.*pp(1,:); c.*pp(2,:); c.*pp(3,:)], zeros(3,1)]; % integral
    nn = 20;        
    m = length(th); th = repmat(th/nn,1,nn); 
    th = reshape(th',m*nn,1);
    th = repmat(th,fix(T/sum(th)),1);
    t1 = [0;cumsum(th)];
    wm = [ polyval(ppi(1,:), t1), ...
           polyval(ppi(2,:), t1), ...
           polyval(ppi(3,:), t1) ];
    wm = diff(wm,1);  % angular increment
    len = fix(length(wm)/2);
    qt = zeros(len, 4);  % quaternion
    q = [1; 0; 0; 0];
    for k=1:len
        wmi = wm(2*k-1:2*k,:);
        phi = (wmi(1,:)+wmi(2,:)+2/3*cross(wmi(1,:),wmi(2,:)))';
        q = qmul(q, rv2q(phi));
        qt(k,:) = q';
    end
    qt = [[1,0,0,0]; qt(nn/2:nn/2:end,:)];
    wm = sumn(wm,nn,1);    
    t = t1(1:nn:nn*length(qt));
    
