function ins = insupdate_compress(ins, imu)
    nn = size(imu,1);
    nts = nn*ins.ts;  nts2 = nts/2;  ins.nts = nts;
    phim = imu(1:3)'; dvbm = imu(4:6)';
    pos01 = ins.pos+ins.Mpv*ins.vn*nts2; vn01= ins.vn+ins.an*nts2; % extrapolation at t1/2
    ins.eth = ethupdate(ins.eth, pos01, vn01);
    ins.wib = phim/nts;  ins.fb = dvbm/nts;  % same as trjsimu
    ins.web = ins.wib - ins.Cnb'*ins.eth.wnie;
    ins.wnb = ins.wib - ins.Cnb'*ins.eth.wnin;
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
%     ins.qnb = qupdt(ins.qnb, ins.wnb*nts);
    ins.qnb = qupdt2(ins.qnb, phim, ins.eth.wnin*nts);
    [ins.qnb, ins.att, ins.Cnb] = attsyn(ins.qnb);
    ins.avp = [ins.att; ins.vn; ins.pos];

