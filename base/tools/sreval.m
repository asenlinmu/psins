function err = sreval(errs, t, vel_dyaw)
% Short-range INS error evaluation. 
%
% Prototype: err = sreval(errs, t)
% Inputs: errs - error source [gyro_drift(deg/h), acc_bias(mg), misalign_ang(deg), init_vel_error(m/s), init_pos_err(m)]
%         t - INS time duration
%         vel_dyaw - INS velocity and init_yaw_error [vel(m/s), dyaw(deg)]
% Output: err - short-range INS error
% 
% Example:
%    err = sreval([10, 1, 0.1, 1, 10], 100);
%
% See also  N/A.

% Copyright(c) 2009-2017, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/06/2017
global glv
    errs(6) = 0;
    eb = errs(1)*glv.dph; db = errs(2)*glv.mg; phi = errs(3)*glv.deg; dv = errs(4); dp = errs(5);
    err = [9.8*t^3*eb/6, [db,9.8*phi]*t^2/2, dv*t, dp];
    if nargin>2
        vel = vel_dyaw(1); dyaw = vel_dyaw(2)*glv.deg;
        err = [err, vel*dyaw*t];
    end
    err = [err, norm(err)];
    figure, bar(err); ylabel(sprintf('INS error / m@%.0fs',t));
    if nargin>2
        xtl = {'eb', 'db', 'phi0', 'v0', 'dpos0', 'dyaw', 'Total'};
    else
        xtl = {'eb', 'db', 'phi0', 'dv0', 'dpos0', 'Total'};
    end
	set(gca, 'XTicklabel', xtl);
    xlabel('Error sources');
