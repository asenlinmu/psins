function inserrplot_polar(avpr, avp, typ)
% Polar navigation error plot
%
% Prototype: inserrplot_polar(avpr, avp, typ)
% Input: avpr - reference AVP
%        avp - avp to compare
%        typ - 'n' for ENU-frame, 'e' for ECEF-frame, 'g' for grid-frame
%
% Example:
%   [imu, avp0, avp] = imupolar_grid(posset(89.9,1.0,100), 100, .1, 200, imuerrset(0,0,10,0));
%   avp0(:,4:5) = avp0(:,4:5)+1;
%   avpn = inspure(imu, avp0, 'V');       inserrplot_polar(avp, avptrans(avpn,'n2g'), 'g');
%   avpe = inspure_ecef(imu, avp0, 'V');  inserrplot_polar(avp, avptrans(avpe,'e2g'), 'g');
%   avpg = inspure_grid(imu, avp0, 'V');  inserrplot_polar(avp, avpg, 'g');
%   inserrplot_polar(avpg, avptrans(avpn,'n2g'), 'g');
%   inserrplot_polar(avpg, avptrans(avpe,'e2g'), 'g');
%
% See also  avp2imu_ecef, imustatic.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/11/2024
global glv
    if nargin<3, typ='g'; end
    avpi = interp1(avpr(:,end), avpr(:,1:9), avp(:,end));
    idx = isnan(avpi(:,1));  avpi(idx,:)=[]; avp(idx,:)=[];
    err = [avp(:,1:9)-avpi, avp(:,end)];
    myfig;
	subplot(3,2,[1,3,5]);  % worldmap('North Pole')?
    if typ=='n', avp=avptrans(avp,'n2e'); avpr=avptrans(avpr,'n2e'); end
    lon = atan2(avp(:,8),avp(:,7)); rho = sqrt(avp(:,8).^2+avp(:,7).^2)/glv.Re/glv.deg;
    polar(lon, rho, 'r'); view(90,90); hold on;
    polar(lon(1), rho(1), '*m');
    lon1 = atan2(avpr(:,8),avpr(:,7)); rho1 = sqrt(avpr(:,8).^2+avpr(:,7).^2)/glv.Re/glv.deg;
    polar(lon1, rho1, 'b--');  % legend('Ref', 'Calcu', 'Orientation','horizontal');
    if typ=='n'
        subplot(322), plot(err(:,end), err(:,1:3)/glv.min), xygo('datt');
        subplot(324), plot(err(:,end), err(:,4:6)), xygo('dV');
        subplot(326), plot(err(:,end), err(:,7:8)/glv.min), xygo('dll');
    elseif typ=='e'
        subplot(322), plot(err(:,end), err(:,1:3)/glv.min), xygo('datt');
        subplot(324), plot(err(:,end), err(:,4:6)), xygo('dV');
        subplot(326), plot(err(:,end), err(:,7:9)), xygo('dxyz');
    elseif typ=='g'
        subplot(322), plot(err(:,end), err(:,1:3)/glv.min), xygo('datt');
        subplot(324), plot(err(:,end), err(:,4:6)), xygo('dV');
        subplot(326), plot(err(:,end), err(:,7:9)), xygo('dxyz');
    end
        
