function [wm, qt, t] = coneSimulatorRandom1(afa, omega, hh)
% output: wm = [          dtheta1, dtheta2, ... , dthetaN ]';
%         qt = [ q0,      q1,      q2,      ... , qN      ]';

    t = cumsum([0;hh(:)]);
    th = diff(t);
    wm = [ -2*sin(afa)*sin(omega*th/2).*sin(omega*(t(1:end-1)+th/2)), ...  % 角增量
            2*sin(afa)*sin(omega*th/2).*cos(omega*(t(1:end-1)+th/2)), ...
           -2*omega*th*sin(afa/2)^2 ];
    qt = [cos(afa/2)*ones(size(t)), ...
        sin(afa/2)*cos(omega*t), ...
        sin(afa/2)*sin(omega*t), ...
        zeros(size(t))]; % 姿态四元数真值

