% clear all
glvs
afa = 1.0*glv.deg;      %半锥角
f = 1;  w = 2*pi*f;     %锥运动频率
th = 0.01;  n = 2;  nth = n*th;     %采样时间, 子样数, 补偿周期
taux = 0.001; tauy = 0.0020; tauz = 0.0020;
time = 60;  %仿真时间长度
len = fix(time/nth);  res = zeros(len,3);  %仿真步数
t = 0;          %初始姿态四元数
qnb_calcu = [cos(afa/2); sin(afa/2)*cos(w*t); sin(afa/2)*sin(w*t); 0];
wm_1 = zeros(1,3);
for k=1:len
    for k1=1:n   %构造n子样
        wm(k1,:) = [-2*sin(afa)*sin(w*th/2)*sin(w*(t+th/2-taux));
            2*sin(afa)*sin(w*th/2)*cos(w*(t+th/2-tauy));
            -2*w*th*(sin(afa/2))^2 ]';  %角增量
        t = t + th;
    end
    qnb_real = [cos(afa/2); sin(afa/2)*cos(w*t); sin(afa/2)*sin(w*t); 0]'; % 姿态四元数真值
    phi = cnscl(wm);         %圆锥误差补偿
    qnb_calcu = qmul(qnb_calcu,rv2q(phi));    %姿态更新
    res(k,:) = qq2phi(qnb_calcu,qnb_real)';  %求姿态误差
end
time = [1:length(res)]*nth;
figure(1)
subplot(3,1,1), plot(time,res(:,1)/glv.sec), ylabel('\it\phi_x\rm / arcsec'); grid on 
subplot(3,1,2), plot(time,res(:,2)/glv.sec), ylabel('\it\phi_y\rm / arcsec'); grid on
subplot(3,1,3), plot(time,res(:,3)/glv.sec), ylabel('\it\phi_z\rm / arcsec'); grid on
xlabel('\itt \rm / s'); 
% 理论圆锥误差
k2 = 1;
for k1=1:n+1
    k2 = k2*(2*k1-1);
end
epsilon = afa^2*(2*pi*f*th)^(2*n+1) * n*factorial(n) / (2^(n+1)*k2);
subplot(3,1,3), hold on, plot([0,time(end)],[0,epsilon*length(time)]/glv.sec,'r--'), hold off;
