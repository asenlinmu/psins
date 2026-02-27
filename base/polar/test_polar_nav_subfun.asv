%
% test_polar_nav_subfun
%% cross
glvs
[imu, avp0, avp] = imupolar_grid(posset(89.9,1.0,100), 100, .01, 200, imuerrset(0,0,0,0));
avp0 = avpadderr(avp0, avperrset([1;1;10],[1;1;0],[1000;500;0]));
avpn = inspure(imu, avp0, 'V');      close(gcf); % inserrplot_polar(avp, avptrans(avpn,'n2g'), 'g');
avpe = inspure_ecef(imu, avp0, 'V'); close(gcf); % inserrplot_polar(avp, avptrans(avpe,'e2g'), 'g');
avpg = inspure_grid(imu, avp0, 'V'); close(gcf); % inserrplot_polar(avp, avpg, 'g');
inserrplot_polar(avpg, avptrans(avpn,'n2g'), 'g');
inserrplot_polar(avpg, avptrans(avpe,'e2g'), 'g');
% inserrplot_polar(avptrans(avp,'g2n'), avpn, 'n');
%% round
[imu, avp0, avp] = imupolar_enu(posset(89.5,0,0), 100, .1, 3000);
avp0 = avpadderr(avp0, avperrset([1;1;10],[1;1;0],[1000;500;0]));
avpn = inspure(imu, avp0, 'V');      close(gcf);
avpg = inspure_grid(imu, avp0, 'V'); close(gcf);
inserrplot_polar(avpg, avptrans(avp,'n2g'), 'g');
inserrplot_polar(avpg, avptrans(avpn,'n2g'), 'g');
%% static
ts=0.01; avp0 = avpset([0;0;0], 0, [89.995; 0; 0]);
imu = imustatic(avp0, ts, 1000, imuerrset(0,0,10,10));
avp = appendt(repmat(avp0',length(imu),1),ts);
avp0 = avpadderr(avp0, avperrset([1;1;10],[1;1;0],[1000;500;0]));
avpn = inspure(imu, avp0, 'V');      close(gcf);
avpg = inspure_grid(imu, avp0, 'V'); close(gcf);
inserrplot_polar(avpg, avptrans(avpn,'n2g'), 'g');
inserrplot_polar(avptrans(avpg,'g2n'), avptrans(avpn,'n2n'), 'n');
inserrplot_polar(avptrans(avpg,'g2n'), avptrans(avpn,'nen'), 'n');

avpnn = avptrans(avpn,'n2nc');
myfig
subplot(321), plot([avpn(:,1:2), avpnn(:,1:2)]);
subplot(322), plot([avpn(:,3), avpnn(:,3)]);
subplot(323), plot([avpn(:,4:6), avpnn(:,4:6)]);
subplot(324), plot([avpn(:,7), avpnn(:,7)]);
subplot(325), plot([avpn(:,8), avpnn(:,8)]);


