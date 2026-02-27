function gcp = gyrocompass11(gcp, imu, vnr, pos, ts)
% 动基座罗经对准(此处只需三个阶段，而不是四阶段)，参见博士后报告P21
global glv
    if isstruct(gcp)==0 % 初始化，调用格式 gcp = gyrocompass(gcp, qnb, vn, pos, ts)
        par = gcp; qnb = imu; vn = vnr;
        clear gcp
        gcp.kk = 1;
        for k=1:gcp.kk
            yaw = (310+(k-1)*360/gcp.kk)*glv.deg;
            gcp.qnb(:,k) = a2qnb([0;0;yaw]);
        end
        gcp.vn = zeros(3,gcp.kk); gcp.pos = pos; gcp.ts = ts;
        gcp.gc1 = gcctrl(par(1));  % 水平调节时间参数Td
        gcp.gc3 = gcctrl(par(2), par(3)); % 方位对准中，水平和方位调节时间参数Td
        gcp.tk = 0; % 计时增量
        gcp.wnc = zeros(3,gcp.kk); gcp.dpos = zeros(3,gcp.kk);
        gcp.eth = earth(gcp.pos);
        gcp.dYaw = zeros(1,gcp.kk);
        return;
    end
    if nargin<2
        for k=1:gcp.kk
            att = q2att(gcp.qnb(:,k))/glv.deg;
            if att(3)<0
                att(3) = att(3)+360;
            end
            gcp.yaw(k,:) = [3+(k-1)*360/gcp.kk, att(3)];
        end
        gcp.yaw(k+1,:) = [gcp.yaw(1,1)+360, gcp.yaw(1,2)];
        for k=1:gcp.kk
            if gcp.yaw(k,2)>gcp.yaw(k,1) && gcp.yaw(k+1,2)<gcp.yaw(k+1,1) && abs(gcp.yaw(k+1,2)-gcp.yaw(k,2)<10)
                b = (gcp.yaw(k+1,2)-gcp.yaw(k,2))/(gcp.yaw(k+1,1)-gcp.yaw(k,1));
                gcp.yy = (gcp.yaw(k,2)-gcp.yaw(k,1)*b)/(1-b);
                gcp.kyy = k;
                break;
            end
        end
        return;
    end
    % gcp = gyrocompass(gcp, imu, vnr)
    vnr = glv.v0;
    nTs = size(imu,1)*gcp.ts;  gcp.tk = gcp.tk + nTs;
    [phim, dvbm] = cnscl(imu); gcp.wbib = phim/nTs;
    for k=1:gcp.kk
        dvn(:,k) = qmulv(gcp.qnb(:,k),dvbm);  % 比力变换
    end
    if gcp.tk<100 % 1,2水平对准
        dVE = gcp.vn(1,:) - vnr(1,:);
        gcp.vn(1,:) = gcp.vn(1,:) + dvn(1,:)-gcp.gc1.kx1*dVE*nTs;
        gcp.dpos(1,:) = gcp.dpos(1,:) + dVE*gcp.gc1.kx3*nTs;
        gcp.wnc(2,:) = dVE*(1+gcp.gc1.kx2)/glv.Re + gcp.dpos(1,:);
        dVN = gcp.vn(2,:) - vnr(2,:);
        gcp.vn(2,:) = gcp.vn(2,:) + dvn(2,:)-gcp.gc1.ky1*dVN*nTs;
        gcp.dpos(2,:) = gcp.dpos(2,:) + dVN*gcp.gc1.ky3*nTs;
        gcp.wnc(1,:) = -dVN*(1+gcp.gc1.ky2)/glv.Re - gcp.dpos(2,:);
        gcp.wnc(3,:) = 0;
    else  % 3方位对准
        dVE = gcp.vn(1,:) - vnr(1,:);
        gcp.vn(1,:) = gcp.vn(1,:) + dvn(1,:)-gcp.gc3.kx1*dVE*nTs;
        gcp.dpos(1,:) = gcp.dpos(1,:) + dVE*gcp.gc3.kx3*nTs;
        gcp.wnc(2,:) = dVE*(1+gcp.gc3.kx2)/glv.Re + gcp.dpos(1,:);
        dVN = gcp.vn(2,:) - vnr(2,:);
        gcp.vn(2,:) = gcp.vn(2,:) + dvn(2,:)-gcp.gc3.kz1*dVN*nTs;
        gcp.wnc(1,:) = -dVN*(1+gcp.gc3.kz2)/glv.Re;
        gcp.wnc(3,:) = (dVN*gcp.gc3.kz3*nTs/gcp.eth.wnie(2)+gcp.wnc(3,:))/(1+gcp.gc3.kz4*nTs);
    end
    gcp.dvn = dvn;
    % 注意计算顺序！
    for k=1:gcp.kk
        Cnb = q2cnb(gcp.qnb(:,k));
        gcp.qnb(:,k) = qmul( gcp.qnb(:,k), rv2q(phim-Cnb'*(gcp.eth.wnin+gcp.wnc(:,k))*nTs) );  % 带反馈控制项wnc的姿态更新
        gcp.dYaw(1,k) = gcp.dYaw(1,k)+gcp.wnc(3,k)*nTs;
    end
