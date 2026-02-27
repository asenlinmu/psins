function ins = insupdate(ins, imu)
% SINS Updating Alogrithm including attitude, velocity and position
% updating.
%
% Prototype: ins = insupdate(ins, imu)
% Inputs: ins - SINS structure array created by function 'insinit'
%         imu - gyro & acc incremental sample(s)
% Output: ins - SINS structure array after updating
%
% See also  insinit, cnscl, earth, trjsimu, imuadderr, avpadderr, q2att,
%           alignvn, aligni0, etm, kffk, kfupdate, insplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 22/03/2008, 12/01/2013, 18/03/2014, 09/09/2014
    nn = size(imu,1);
    nts = nn*ins.ts;  nts2 = nts/2;  ins.nts = nts;
%     [phim, dvbm] = cnscl(imu);    % coning & sculling compensation
    [phim, dvbm] = cnscl0(imu);    % coning & sculling compensation
%     phim = ins.Kg*phim-ins.eb*nts; dvbm = ins.Ka*dvbm-ins.db*nts;  % calibration
    pos01 = ins.pos+ins.Mpv*ins.vn*nts2; vn01= ins.vn+ins.an*nts2; % extrapolation at t1/2
    ins.eth = ethupdate(ins.eth, pos01, vn01);
    ins.wdot = mean(diff([ins.wm0; imu(:,1:3)]),1)'/(ins.ts*ins.ts); ins.wm0 = imu(:,1:3);
    ins.wib0 = ins.wib1;  ins.wib1 = phim/nts;  ins.wdot1 = (ins.wib1-ins.wib0)/nts;
    ins.wib = ins.wib1 + ins.Kwdot.*ins.wdot;  ins.fb = dvbm/nts;  % same as trjsimu
    ins.web = ins.wib - ins.Cnb'*ins.eth.wnie;
    ins.wnb = ins.wib - ins.Cnb'*ins.eth.wnin;
%     K = eye(3)+0*0.0013*askew(ins.wdot);
%     w = ins.wdot;
%     K = eye(3)-0.02*[0,w(2),w(3); w(1),0,w(3); w(1),w(2),0];
    K = [-0.000273287081470337
      0.000222005483398283
      3.30867976586519e-05]*0;
    K = eye(3)-askew(K.*ins.wdot);
    phim = K*phim;
    %% (1)velocity updating
    ins.fn = qmulv(ins.qnb, ins.fb);
%     ins.an = qmulv(rv2q(-ins.eth.wnin*nts2),ins.fn) + ins.eth.gcc;
    ins.an = rotv(-ins.eth.wnin*nts2, ins.fn) + ins.eth.gcc;
    vn1 = ins.vn + ins.an*nts;
    %% (2)position updating
%     ins.Mpv = [0, 1/ins.eth.RMh, 0; 1/ins.eth.clRNh, 0, 0; 0, 0, 1];
    ins.Mpv(4)=1/ins.eth.RMh; ins.Mpv(2)=1/ins.eth.clRNh;
    ins.Mpvvn = ins.Mpv*(ins.vn+vn1)/2;
    ins.pos = ins.pos + ins.Mpvvn*nts;  
    ins.vn = vn1;
    %% (3)attitude updating
    ins.Cnb0 = ins.Cnb;
%     ins.qnb = qupdt(ins.qnb, ins.wnb*nts);
    ins.qnb = qupdt2(ins.qnb, phim, ins.eth.wnin*nts);
    [ins.qnb, ins.att, ins.Cnb] = attsyn(ins.qnb);
    ins.avp = [ins.att; ins.vn; ins.pos];
    %% lever-arm and time-delay compensation
    ins.MpvCnb = ins.Mpv*ins.Cnb;  ins.CW = ins.Cnb*askew(ins.web);
    %ins.attLD = q2att(qmul(ins.qnb, rv2q(ins.wnb*ins.tDelay)));
    ins.vnLD = ins.vn + ins.CW*ins.lever + ins.an*ins.tDelay;
    ins.posLD = ins.pos + ins.MpvCnb*ins.lever + ins.Mpvvn*ins.tDelay;
