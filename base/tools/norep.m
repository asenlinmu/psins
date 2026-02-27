function res = norep(scr, clm)
% To get non-repeat data for specified columns.
%
% Prototype: res = norep(scr, clm)
% Inputs: scr - data source input
%         clm - column for non-repeat
% Output: res - result
%
% See also  no0, setrep0, normv.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/09/2020
    if nargin==2,   nv = normv(scr(:,clm));
    else,          nv = normv(scr); end
    nv = diff([nv(1)-1;nv]);
    res = scr(abs(nv)>0, :);
