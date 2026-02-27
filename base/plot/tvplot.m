function tvplot(xyz)
% Three-view drawing plot
%
% Prototype: tvplot(xyz)
% Input: xyz - 3-D data input (X-East/Y-North/Z-Up, in m), 
% Example: 
%     t = (0:0.1:10*pi)'; afa = 2*pi*0.1*t; tvplot([sin(afa), cos(afa), t]);
%
% See also  pos2dplot.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/05/2020
    myfigure,
    subplot(221), plot(xyz(:,1), xyz(:,3)); xygo('East / m', 'Up / m'); title('South->North View');
    subplot(222), plot(xyz(:,2), xyz(:,3)); xygo('North / m', 'Up / m'); title('West->East View');  set(gca,'XDir','reverse')
    subplot(223), plot(xyz(:,1), xyz(:,2)); xygo('East / m', 'North / m'); title('Up->Down View');

