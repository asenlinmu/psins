function [wm, qt, wt] = coneSimulatorRandom(afa, omega, hh, T)
% output: wm = [          dtheta1, dtheta2, ... , dthetaN ]';
%         qt = [ q0,      q1,      q2,      ... , qN      ]';
%         wt = [ w0,      w1,      w2,      ... , wN      ]';
    Tm = sum(hh); nn = length(hh);
    t = 0:Tm:T; t = repmat(t,nn,1);
    for k=2:nn
        t(k,:) = t(k,:) + sum(hh(1:k-1));
    end
    t = reshape(t, size(t,1)*size(t,2),1);
    th = diff(t);
    wm = [ -2*sin(afa)*sin(omega*th/2).*sin(omega*(t(1:end-1)+th/2)), ...  % 角增量
            2*sin(afa)*sin(omega*th/2).*cos(omega*(t(1:end-1)+th/2)), ...
           -2*omega*th*sin(afa/2)^2 ];
    qt = [cos(afa/2)*ones(size(t)), ...
        sin(afa/2)*cos(omega*t), ...
        sin(afa/2)*sin(omega*t), ...
        zeros(size(t))]; % 姿态四元数真值
    if nargout==3
        wt = [ -omega*sin(afa)*sin(omega*t), ...  % angular rate
                omega*sin(afa)*cos(omega*t), ...
               -2*omega*sin(afa/2)^2*ones(size(t)) ];
        % plot([wm(:,:)/th- (wt(1:end-1,:)+wt(2:end,:))/2]);
    end

