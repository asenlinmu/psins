function [att0, res] = aligni0vb(imu, vb, pos, ts)
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/08/2014
global glv
    if nargin<3,  ts = imu(2,7)-imu(1,7);  end
    if size(vb,2)==1, vb=[zeros(size(vb)), vb, zeros(size(vb))]; end
    nn = 4; nts = nn*ts;  ratio = 1;
    len = fix(length(imu)/nn)*nn;
    eth = earth(pos);  lat = pos(1);  g0 = -eth.gn(3);
    qib0b = [1; 0; 0; 0];  qi0ib0 = qib0b;
    [vib0, vi0, pib0, vvib0, vpib0, ivpib0, iivpib0, pi0, vib0_1, vi0_1] = setvals(zeros(3,1));
    [pib0k, vpib0k, ivpib0k, iivpib0k, pi0k, vi0k, vib0k, fi0k, fib0k, attk] = prealloc(len/nn, 3);
    k0 = fix(5/ts); % exculde the first 5s
    ki = timebar(nn, len, 'Initial align based on inertial frame.');
    for k=1:nn:len-nn+1
        wvm = imu(k:k+nn-1, 1:6);  kts = (k+nn-1)*ts;
        [phim, dvbm] = cnscl(wvm);
        fib0 = qmulv(qib0b, dvbm)/nts;   % f
        vib0 = vib0 + fib0*nts;          % vel
        pib0 = ratio*pib0 + (vib0_1+vib0)*nts/2;  vib0_1 = vib0; % pos
        vvib0 = vvib0 + (vb(k+nn-1,:)-vb(k,:))';
        vpib0 = vpib0 + qmulv(qib0b, vvib0)*nts;
        ivpib0 = ivpib0 + vpib0*nts;
        iivpib0 = iivpib0 + ivpib0*nts;
        fi0 = [eth.cl*cos(kts*glv.wie);eth.cl*sin(kts*glv.wie);eth.sl]*g0;
        vi0 = vi0 + fi0*nts;
        pi0 = ratio*pi0 + (vi0_1+vi0)*nts/2;      vi0_1 = vi0;
        qib0b = qupdt(qib0b, phim);  % qib0b updating
        pib0k(ki,:) = pib0'; vib0k(ki,:) = vib0'; fib0k(ki,:) = fib0'; % recording
        vpib0k(ki,:) = vpib0'; ivpib0k(ki,:) = ivpib0'; iivpib0k(ki,:) = iivpib0'; 
        pi0k(ki,:) = pi0';   vi0k(ki,:) = vi0';   fi0k(ki,:) = fi0';
        if k>k0
            k1 = fix(ki/2);
            swiet = sin(kts*glv.wie); cwiet = cos(kts*glv.wie);
            Cni0 = [-swiet,cwiet,0; 
                -eth.sl*cwiet,-eth.sl*swiet,eth.cl; 
                eth.cl*cwiet,eth.cl*swiet,eth.sl];
            qni0 = m2qua(Cni0);
            wib0ie = qmulv(qconj(qi0ib0),Cni0'*eth.wnie);
            p0 = vpib0k(k1,:)-pib0k(k1,:)+cross(wib0ie,ivpib0k(k1,:))+iivpib0k(k1,:)*g0/glv.Re;
            p1 = vpib0-pib0+cross(wib0ie,ivpib0)+iivpib0*g0/glv.Re;
            qi0ib0 = dv2atti(pi0k(k1,:)', pi0, -p0', -p1);
            qnb = qmul(qmul(qni0,qi0ib0),qib0b);
            attk(ki,:) = q2att(qnb)';     % using pos
       end
       ki = timebar;
    end
    k0 = fix(k0/nn)+1;
    Cni0 = [0,1,0; -eth.sl,0,eth.cl;  eth.cl,0,eth.sl];
    att0 = q2att(qmul(m2qua(Cni0),qi0ib0));
    attk(1:k0,:) = repmat(att0',k0,1);
    res = varpack(lat, nts, vib0k, pib0k, fib0k, vi0k, pi0k, fi0k, attk, att0); 
    att0 = attk(end,:)';
    resdisp('Initial align attitudes (arcdeg)', att0/glv.deg);

    ai0plot(nts, attk);
    
function ai0plot(ts, attk)
global glv
    t = (1:length(attk))'*ts;
    myfigure;
    subplot(211), plot(t, attk(:,1:2)/glv.deg), xygo('pr');
    subplot(212), plot(t, attk(:,3)/glv.deg), xygo('y');