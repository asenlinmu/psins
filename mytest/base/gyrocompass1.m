function gcp = gyrocompass(gcp, wm, vm, vnr, ts)
% 动基座罗经对准(此处只需三个阶段，而不是四阶段)，参见博士后报告P21
global glv
    if isstruct(gcp)==0 % 初始化，调用格式 gcp = gyrocompass(gcp, qnb, vn, pos)
        par = gcp; qnb = wm; vn = vm; pos = vnr;
        clear gcp
        gcp.qnb = qnb; gcp.vn = vn; gcp.pos = pos; gcp.ts = ts;
        gcp.qnb1 = gcp.qnb; gcp.vn1 = gcp.vn;
        gcp.gc1 = gcctrl(par(1));  % 水平调节时间参数Td
        gcp.gc3 = gcctrl(par(2), par(3)); % 方位对准中，水平和方位调节时间参数Td
        gcp.tn = par(4:5); % 1，2阶段的对准时间
        gcp.init2 = par(6);  % 是否需要方位粗略自对准的初始化 0--需要
        gcp.modify = 1;
        gcp.vnorb = par(7); % 外参考速度是n系还是b系 0--b系
        gcp.tk = 0; % 计时增量
        gcp.wnc = glv.v0; gcp.dpos = glv.v0;
        return;
    end
    % gcp = gyrocompass(gcp, wm, vm, vnr)
    if nargin<4  % 外参考速度为0，即静基座对准
        vnr = glv.v0;
    elseif gcp.vnorb == 0
        vnr = qmulv(gcp.qnb,vnr);
    end
    nTs = size(wm,2)*gcp.ts;  gcp.tk = gcp.tk + nTs;
    [phim, dvbm] = cnscl(wm,vm);
    dvn = qmulv(gcp.qnb, dvbm);  % 比力变换
    gcp.eth = earth(gcp.pos,gcp.vn);
    if gcp.tk<gcp.tn(2) % 1,2水平对准
        dVE = gcp.vn(1) - vnr(1);
        gcp.vn(1) = gcp.vn(1) + dvn(1)-gcp.gc1.kx1*dVE*nTs;
        gcp.dpos(1) = gcp.dpos(1) + dVE*gcp.gc1.kx3*nTs;
        gcp.wnc(2,1) = dVE*(1+gcp.gc1.kx2)/glv.Re + gcp.dpos(1);
        dVN = gcp.vn(2) - vnr(2);
        gcp.vn(2) = gcp.vn(2) + dvn(2)-gcp.gc1.ky1*dVN*nTs;
        gcp.dpos(2) = gcp.dpos(2) + dVN*gcp.gc1.ky3*nTs;
        gcp.wnc(1,1) = -dVN*(1+gcp.gc1.ky2)/glv.Re - gcp.dpos(2);
        gcp.wnc(3,1) = 0;
        if gcp.tk>gcp.tn(1) 
            if ~gcp.init2
                gcp.init2 = 1; gcp.modify = 0;
                gcp.qnb1 = gcp.qnb; gcp.vn1 = gcp.vn;
            end
            gcp.vn1 = gcp.vn1 + qmulv(gcp.qnb1, dvbm);  % 自由速度导航
            gcp.qnb1 = qmul( gcp.qnb1, rv2q(phim-qmulv(qconj(gcp.qnb1),gcp.eth.wnin)*nTs) );
        end
    else  % 3方位对准
        if ~gcp.modify
            gcp.modify = 1;
            tt = gcp.tn(2)-gcp.tn(1);
            dvn1 = gcp.vn1 - gcp.vn;
            ss = -2*dvn1(2)/(9.8*gcp.eth.wnie(2)*tt^2); cc = 2*dvn1(1)/(9.8*gcp.eth.wnie(2)*tt^2) + 1;
            sc = sqrt(ss^2+cc^2); ss = ss/sc; cc = cc/sc; Cbb = [cc,-ss,0; ss,cc,0; 0,0,1]; 
            att = q2att(gcp.qnb); Cnb = q2cnb(gcp.qnb)*Cbb; att1 = m2att(Cnb); % 水平姿态保留
            gcp.qnb = a2qnb([att(1:2,:);att1(3,1)]); gcp.vn = Cbb*gcp.vn; gcp.dpos = Cbb*gcp.dpos; % 状态更改！
        end
        dVE = gcp.vn(1) - vnr(1);
        gcp.vn(1) = gcp.vn(1) + dvn(1)-gcp.gc3.kx1*dVE*nTs;
        gcp.dpos(1) = gcp.dpos(1) + dVE*gcp.gc3.kx3*nTs;
        gcp.wnc(2,1) = dVE*(1+gcp.gc3.kx2)/glv.Re + gcp.dpos(1);
        dVN = gcp.vn(2) - vnr(2);
        gcp.vn(2) = gcp.vn(2) + dvn(2)-gcp.gc3.kz1*dVN*nTs;
        gcp.wnc(1,1) = -dVN*(1+gcp.gc3.kz2)/glv.Re;
        gcp.wnc(3,1) = (dVN*gcp.gc3.kz3*nTs/gcp.eth.wnie(2)+gcp.wnc(3,1))/(1+gcp.gc3.kz4*nTs);
    end
    % 注意计算顺序！
    gcp.pos = gcp.pos + nTs*[vnr(2)/gcp.eth.RMh;vnr(1)/(gcp.eth.RNh*gcp.eth.cl);vnr(3)]; % 定位更新
    gcp.qnb = qmul( gcp.qnb, rv2q(phim-qmulv(qconj(gcp.qnb),gcp.eth.wnin+gcp.wnc)*nTs) );  % 带反馈控制项wnc的姿态更新
