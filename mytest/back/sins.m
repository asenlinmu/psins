function ss = sins(ss, imu, var1, var2)
% SINS Updating Alogrithm.
% Initialization Methods: 
%                 ss = sins(avp0, ts);
%                 ss = sins(avp0, ts, avperr);
%                 ss = sins(qnb0, vn0, pos0, ts);
% SINS Updating:  ss = sins(ss, imu);
%
% See also  cnscl, earth, sinsstatic, imuadderr, avpadderr, q2att,
%           alignvn, aligni0, etm, kffk, kffilt, navplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/01/2013
    %% %%%%%%%%%%%%%%%%%% SINS Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%
    if isstruct(ss)==0
        if nargin==2
            [avp0, ts] = setvals(ss(:), imu);
            [qnb0, vn0, pos0] = setvals(a2qua(avp0(1:3)), avp0(4:6), avp0(7:9));
        elseif nargin==3
            [avp0, ts] = setvals(avpadderr(ss(:),var1), imu);
            [qnb0, vn0, pos0] = setvals(a2qua(avp0(1:3)), avp0(4:6), avp0(7:9));
        elseif nargin==4
            [qnb0, vn0, pos0, ts] = setvals(ss(:), imu, var1, var2);
        end
        ss = initsins(qnb0, vn0, pos0, ts);
        return;
    end
    %% %%%%%%%%%%%%%%%%%%%%%% SINS Updating %%%%%%%%%%%%%%%%%%%%%%%%%%%
    nn = size(imu,1);  
    nts = ss.ts*nn;  nts2 = nts/2;  ss.nts = nts;
    [phim, dvbm] = cnscl(imu);    % coning & sculling compensation
    phim = ss.Kg*phim-ss.eb*nts; dvbm = ss.Ka*dvbm-ss.db*nts;
%     ss.eth = earth(ss.pos, ss.vn);
    ss.eth = earth(ss.pos+ss.M5*ss.vn*nts2, ss.vn+ss.an*nts2);
    ss.Cnb = q2mat(ss.qnb);  % ss.qnb may be modified in KF, so reset ss.Cnb
% 	Cb01n = q2mat(qmul(ss.qnb,rv2q(phim/2)))';  % mean DCM estimation for wnin transform, good !!! 17/03/2014
    Cnb1 = q2mat(qmul(ss.qnb,rv2q(phim)));
    ss.wib = phim/nts; ss.web = ss.wib - ss.Cnb'*ss.eth.wnie;
%     ss.wnb = ss.wib-Cb01n*ss.eth.wnin;   ss.fb = dvbm/nts;
    ss.wnb = ss.wib-(ss.Cnb+Cnb1)'*ss.eth.wnin/2;   ss.fb = dvbm/nts;  % same as trjsimu
    ss.an = qmulv(rv2q(-ss.eth.wnin*nts2),ss.Cnb*ss.fb) + ss.eth.gcc;
    vn1 = ss.vn + ss.an*nts;    % velocity updating
    ss.M5 = [0, 1/ss.eth.RMh, 0; 1/ss.eth.clRNh, 0, 0; 0, 0, 1];
    pos1 = ss.pos + ss.M5*((ss.vn+vn1)*nts2);  % position updating
    ss.vn = vn1;  ss.pos = pos1;
    ss.qnb = qmul(ss.qnb, rv2q(ss.wnb*nts));   % attitude updating
    nq2 = ss.qnb'*ss.qnb;  % norm(qnb)^2
    if (nq2>1.00001||nq2<0.99999),  ss.qnb = ss.qnbsqrt(nq2);  end % normalization
    [ss.qnb, ss.att, ss.Cnb] = attsyn(ss.qnb);
    ss.avp = [ss.att; ss.vn; ss.pos];
    %% %%%%%%%%%% lever-arm and time-delay compensation %%%%%%%%%%%%%%%
    %ss.attLD = q2att(qmul(ss.qnb, rv2q(ss.wnb*ss.tDelay)));
    ss.vnLD = ss.vn + ss.Cnb*cross(ss.web,ss.lever) + ss.an*ss.tDelay;
    ss.posLD = ss.pos + ss.M5*(ss.Cnb*ss.lever+ss.vn*ss.tDelay);

    
function ss = initsins(qnb0, vn0, pos0, ts)
	ss = [];
	ss.qnb  = qnb0; ss.vn  = vn0; ss.pos  = pos0; ss.avp  = [q2att(qnb0); vn0; pos0];
	[ss.qnb, ss.att, ss.Cnb] = attsyn(ss.qnb); 
	ss.eth = earth(ss.pos, ss.vn);
	% 'M5,web,wib,an' may be very useful outside SINS, so we calucate and save them.
	ss.M5 = [0, 1/ss.eth.RMh, 0; 1/ss.eth.clRNh, 0, 0; 0, 0, 1];
	[ss.wnb, ss.web, ss.wib, ss.an, ss.eb, ss.db, ss.lever] = setvals(zeros(3,1));
	ss.fb = -ss.Cnb'*ss.eth.gn;
	ss.Kg = eye(3); ss.Ka = eye(3);
	ss.tDelay = 0;
	ss.ts = ts; ss.nts = ts;
