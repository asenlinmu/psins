function data = adddt(data, dt)
% Add the time tag data(:,end) with dt.
%
% Prototype: data = adddt(data, dt)
%
% See also  getat, sortt, tshift.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/11/2020
    data(:,end) = data(:,end)+dt;