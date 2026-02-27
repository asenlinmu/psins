function q = rv2q1(rv)
% Convert rotation vector to transformation quaternion.
%
% Prototype: q = rv2q(rv)
% Input: rv - rotation vector
% Output: q - corresponding transformation quaternion, such that
%            q = [ cos(|rv|/2); sin(|rv|/2)/|rv|*rv ]
% 
% See also  q2rv, rv2m, m2rv, a2qua, rotv, qupdt.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/02/2009, 22/05/2014
    n2 = rv'*rv;
    if n2>1.0e-16  % if n=1e-8 then sin(n/2)/n-1/2==0, cos(n/2)-1==0
        n = sqrt(n2);
        n_2 = n/2;
        q = [cos(n_2); sin(n_2)/n*rv];
    else
        q = [1; 0.5*rv];
    end
