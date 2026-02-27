function [wm, qt, att] = swaysimu(afa, f, phase0, ts, T)
% Sway simulation with x-,y-,z-axis.
%
% Prototype: [wm, qt] = swaysimu(afa, f, phase0, ts, T)
% Inputs: afa - sway amplitude (in degree)
%        f - sway frequency (in Hz)
%        phase - sway initial-phase (in degree)
%        ts - sampling interval
%        T - total simulation time
% Outputs: wm  = [      wm1,  wm2, ... , wmN ]';    % angular increment
%          qt  = [ q0,  q1,   q2,  ... , qN  ]';    % quaternion reference
%          att = [att0, att1, att2, ..., attN]';
%
% See also  conesimu.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/08/2013
    if length(afa)==1, afa=[afa;afa;afa]; end
    if length(f)==1, f=[f;f;f]; end
    if length(phase0)==1, phase0=[phase0;phase0;phase0]; end
    if nargin<5,  T = 10/min(f);  end  % the default T is 10 cycles
    afa = afa*pi/180;
    omega = 2*pi*f;
    phase0 = phase0*pi/180;
    n10=10; t = (0:ts/10:T)'; len = length(t);
    wm = [afa(1)*sin(omega(1)*t+phase0(1)), ...      % angular increment
          afa(2)*sin(omega(2)*t+phase0(2)), ...
          afa(3)*sin(omega(3)*t+phase0(3))];
    wm = diff(wm);
    q0 = [1;0;0;0]; qt = repmat(q0', fix(len/n10), 1);   % quaternion update
    nn = 2; ki = 2;
    for k=1:2:len-1
        k1 = k+nn-1;
        phim = cnscl(wm(k:k1,:));
        q0 = qupdt(q0,phim);
        qt(ki,:) = q0'; ki = ki+1;
    end
    wm = sumn(wm,n10);
    qt = qt(1:n10/2:end,:);
    if nargout>2                    % Euler angle
        att = qt;
        for k=1:length(qt)
            att(k,:) = [q2att(qt(k,:)'); (k-1)*n10*ts]';
        end
    end
