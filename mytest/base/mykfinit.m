function kf = mykfinit(nts, varargin)
% Called by kfinit, for private use only.
%
% See also  kfinit.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/10/2013
global glv kftype
[Re,deg,dph,ug,mg] = ... % just for short
    setvals(glv.Re,glv.deg,glv.dph,glv.ug,glv.mg); 
o33 = zeros(3); I33 = eye(3); 
kf = [];
switch(kftype.curtype)
    case {kftype.leador_POS_fusion_34}
        kf.fkno = 34;  kf.hkno = 3;
        dKga = [20,10,10, 10,20,10, 10,10,20,  20,10,10, 20,10, 20]';
        imuerr = imuseterr([0.1, 0.01, 1000, 10], dKga);
        avpErr = [ [1;1;1]*deg; [1;1;1]; [[.91;.91]/Re;.91] ];
        Qt = diag([[3;3;3]*dph; [5;5;5]*mg; zeros(9+3+1+15,1)])^2;
        kf.Qk = Qt*nts; dim = length(kf.Qk);
        kf.Rk = diag(avpErr(7:9))^2;
        kf.Pxk = diag([avpErr; imuerr.eb; imuerr.db; [10;10;10]; 0.01; imuerr.dKga]*10)^2;
        kf.Hk = zeros(3,dim); kf.Hk(:,7:9) = I33;
        kf.xtau(1:dim,1) = 0;
        kf.xtau(1:15) = [10;10;10; 3;3;3; 10;10;10; 10;10;10; 10;10;1.0]*10;
    case {kftype.test_POS_fusion_34}
        avpErr = var1; imuerr = var2; lever = var3; dT = var4;
        Qt = diag([imuerr.web; imuerr.wdb; zeros(9+3+1+15,1)])^2;
        kf.Qk = Qt*nts; dim = length(kf.Qk);
        kf.Rk = diag(avpErr(7:9))^2;
        kf.Pxk = diag([avpErr; imuerr.eb; imuerr.db; lever; dT; imuerr.dKga]*10)^2;
        kf.Hk = zeros(3,dim); kf.Hk(:,7:9) = I33;
        kf.xtau(1:dim,1) = 0;
        %kf.xtau(1:15) = [10;10;10; 3;3;3; 3;3;3; 10;10;10; 10;10;10];
    case kftype.leador_POS_fusion_19,
        imuerr = imuSetErr([0.1, 0.01, 1000, 100]);
        avpErr = [ [1;1;1]*deg; [1;1;1]; [[10;10]/Re;10] ];
        Qt = diag([[3;3;3]*dph; [10;10;10]*mg; zeros(9+3+1,1)])^2;
        kf.Qk = Qt*nts;
        kf.Rk = diag([[.5;.5]/Re;.5])^2;
        kf.Pxk = diag([avpErr; imuerr.eb; imuerr.db; [10;10;10]; 0.01])^2 * 1;
        kf.Hk = zeros(3,length(Qt)); kf.Hk(:,7:9) = I33;
        kf.xtau = inf(length(kf.Pxk),1);    
        kf.xtau(1:15) = [10;10;10; 1;1;1; 1;1;1; 10;10;10; 10;10;10];
    case {kftype.test_SINS_GPS_19, kftype.test_POS_fusion_19},
        avpErr = var1; imuerr = var2;
        Qt = diag([imuerr.web; imuerr.wdb; zeros(9+3+1,1)])^2;
        kf.Qk = Qt*nts;
        kf.Rk = diag(avpErr(7:9))^2;
        kf.Pxk = diag([avpErr(1:6); [10;10]/Re;10; imuerr.eb; imuerr.db; [10;10;10]; 0.1])^2;
        kf.Hk = zeros(3,length(Qt)); kf.Hk(:,7:9) = I33;
        kf.xtau = inf(length(kf.Pxk),1);    
        kf.xtau(1:15) = [10;10;10; 1;1;1; 1;1;1; 10;10;10; 10;10;10];
    case kftype.leador_ins_od_dt,
        imuerr = imuSetErr([0.5, 0.1, 1000, 100]);
        avpErr = [ [1;1;5]*deg; [1;1;1]; [[10;10]/Re;10]*10 ];
        Qt = diag([imuerr.web; imuerr.wdb; zeros(9+6+1,1)])^2;
        kf.Qk = Qt*nts;
        kf.Rk = diag([0.1;0.1;0.1])^2;
        kf.Pxk = diag([avpErr; imuerr.eb; imuerr.db; [1*deg;0.1;1*deg]; [5;5;5]; 0.1])^2 * 1;
        kf.Hk = zeros(3,length(Qt)); 
    case kftype.leador_ins_od,
        imuerr = imuSetErr([0.5, 0.1, 1000, 100]);
        avpErr = [ [1;1;5]*deg; [1;1;1]; [[10;10]/Re;10]*10 ];
        Qt = diag([imuerr.web; imuerr.wdb; zeros(9+6,1)])^2;
        kf.Qk = Qt*nts;
        kf.Rk = diag([0.1;0.1;0.1])^2;
        kf.Pxk = diag([avpErr; imuerr.eb; imuerr.db; [1*deg;0.1;1*deg]; [5;5;5]])^2 * 1;
        kf.Hk = zeros(3,length(Qt)); 
    case kftype.wangjun_ins_gps_19,
        avpErr = var1; imuerr = var2;
        Qt = diag([imuerr.web; imuerr.wdb; zeros(13,1)])^2;
        kf.Qk = Qt*nts;
        kf.Rk = diag(avpErr(7:9))^2;
        kf.Pxk = diag([avpErr*10; imuerr.eb; imuerr.db; [10;10;10]; 0.01])^2 * 1;
        kf.Hk = [zeros(3,6),eye(3),zeros(3,10)];
        kf.xtau = [0;0;0; 10;10;10; 10;10;10; 0;0;0; 0;0;0; inf;inf;inf; inf];
    case kftype.WangJun_POS_fusion_34,
        avpErr = var1; imuerr = var2;
        Qt = diag([imuerr.web; imuerr.wdb; zeros(9+3+1+15,1)])^2;
        kf.Qk = Qt*nts;
        kf.Rk = diag([[1;1]/glv.Re;1])^2;
        kf.Pxk = diag([avpErr; imuerr.eb; imuerr.db; [10;10;10]; 0.01; imuerr.dKga])^2 * 1;
        kf.Hk = zeros(3,length(Qt)); kf.Hk(:,7:9) = I33;
    otherwise,
        error('kftype mismatch in mykfinit');
end
