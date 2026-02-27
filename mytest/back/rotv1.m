function vo = rotv1(rv, vi)
% Rotate a vector by some rotation vector.
%
% Prototype: vo = rotv(rv, vi)
% Inputs: rv - rotation vector
%         vi - input vector to be rotated
% Output: vo - output vector result, such that vo=rv2m(rv)*vi
% 
% See also  rv2m, rv2q, qmulv, m2rv, q2rv, a2mat.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/06/2014

	xx = rv(1)*rv(1); yy = rv(2)*rv(2); zz = rv(3)*rv(3);
	n2 = xx+yy+zz;
    if n2<1.e-8
        a = 1-n2*(1/6-n2/120); b = 0.5-n2*(1/24-n2/720);  % a->1, b->0.5
    else
        n = sqrt(n2);
        a = sin(n)/n;  b = (1-cos(n))/n2;
    end
	arvx = a*rv(1);  arvy = a*rv(2);  arvz = a*rv(3);
	bxx = b*xx;  bxy = b*rv(1)*rv(2);  bxz = b*rv(1)*rv(3);
	byy = b*yy;  byz = b*rv(2)*rv(3);  bzz = b*zz;
	% m = I + a*(rvx) + b*(rvx)^2;
	m1=1     -byy-bzz; m4= -arvz+bxy;     m7=  arvy+bxz;
	m2=  arvz+bxy;     m5=1     -bxx-bzz; m8= -arvx+byz;
	m3= -arvy+bxz;     m6=  arvx+byz;     m9=1     -bxx-byy;
    vo = vi;    % vo = m*vi;
    vo(1) = m1*vi(1)+m4*vi(2)+m7*vi(3);
    vo(2) = m2*vi(1)+m5*vi(2)+m8*vi(3);
    vo(3) = m3*vi(1)+m6*vi(2)+m9*vi(3);
