function m = rv2m1(rv)
% Convert rotation vector to transformation matrix.
%
% Prototype: m = rv2m1(rv)
% Input: rv - rotation vector, make sure |rv| is small.
% Output: m - corresponding DCM, such that
%     m ~= I + rvx
%     where rvx is the askew matrix or rv.
% 
% See also  m2rv, rv2q, q2rv, a2mat.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/02/2009
    m = [ 1,    -rv(3), rv(2); 
          rv(3), 1,    -rv(1); 
         -rv(2), rv(1), 1 ];

