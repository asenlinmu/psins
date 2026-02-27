function [trj, avp0] = trjInverse(apt, ts, t0)
global glv
    % 在指定的采样点处插值生成avp
    if nargin<3
        t0 = apt(1,end);
    end
    t = (t0:ts:apt(end,end))';
    apt(:,3) = yawContinuous(apt(:,3));
    avp = zeros(length(t), 9);
    for k=1:3  % 姿态
        avp(:,k) = spline(apt(:,end), apt(:,k), t);
    end
    for k=1:3
        pp = spline(apt(:,end), apt(:,3+k));
        avp(:,6+k) = ppval(pp, t); % 位置
        dpp = pp;
        for kk=1:pp.pieces
            dpp.coefs(kk,:) = [0, [3,2,1].*pp.coefs(kk,1:3)]; % 三次多项式微分系数
        end
        avp(:,3+k) = ppval(dpp, t); % 速度
    end
    % 生成imu信息
    len = length(t);
    wvm = zeros(len, 6); wm_1 = [0;0;0];
    avp0 = avp(1,:)';
    Cbn_1 = a2cnb(avp0(1:3))';    
    pos_1 = avp0(7:9);    eth = earth(pos_1);
    vn_1 = [avp0(5)*eth.clRNh; avp0(4)*eth.RMh; avp0(6)]; avp0(4:6) = vn_1';
    timebar(1,len,'Trajectory inversion.');
    for k=2:len  % begin form 2
        vn = [avp(k,5)*eth.clRNh; avp(k,4)*eth.RMh; avp(k,6)];
        pos = avp(k,7:9)';
        eth = earth((pos_1+pos)/2, (vn_1+vn)/2);
        vn = [avp(k,5)*eth.clRNh; avp(k,4)*eth.RMh; avp(k,6)]; avp(k,4:6) = vn;
        dvbm = Cbn_1*qmulv(rv2q(eth.wnin*(ts/2)), vn-vn_1-eth.gcc*ts); % sins
        Cnb = a2cnb(avp(k,1:3));
        wbts = q2rv(qmul(m2qnb(Cbn_1),m2qnb(Cnb)));
        phim = wbts + (Cbn_1+Cnb')*(eth.wnin*ts/2); 
        wm = (glv.I33-askew(wm_1*(1.0/12)))*phim; % using previous subsample: phim = (wm + 1/12*cross(wm_1,wm))
        vm = (glv.I33-askew(wm*(1.0/2)))*dvbm;
        wvm(k,:) = [wm; vm]'; % 单子样
        Cbn_1 = Cnb'; vn_1 = vn; pos_1 = pos; wm_1 = wm;
        timebar;
    end
    trj = [avp(2:end,:), wvm(2:end,:), t(2:end)];

function yaw = yawContinuous(yaw)
% 将方位角变连续
    df = diff(yaw);
    g = find(df>pi);
    s = find(df<-pi);
    for k=1:length(g)
        yaw(g(k)+1:end) = yaw(g(k)+1:end) - 2*pi;
    end
    for k=1:length(s)
        yaw(s(k)+1:end) = yaw(s(k)+1:end) + 2*pi;
    end
