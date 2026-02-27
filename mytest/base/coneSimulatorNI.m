function [wm, qt, t] = coneSimulatorNI(afa, omega, th, T)
% coning motion simulation for non-uniform sampling interval
% output: wm = [          dtheta1, dtheta2, ... , dthetaN ]';
%         qt = [ q0,      q1,      q2,      ... , qN      ]';
%          t = [ t0,      t1,      t2,      ... , tN      ]';
    len = fix(T/sum(th));
    th = repmat(th,len,1); 
    t = cumsum(th); t = [0;t(1:end-1)];
    wm = [ -2*omega*th*sin(afa/2)^2, ...
           -2*sin(afa)*sin(omega*th/2).*sin(omega*(t+th/2)), ...  % 角增量
            2*sin(afa)*sin(omega*th/2).*cos(omega*(t+th/2))  ];
    wm(end,:) = [];       
    qt = [cos(afa/2)*ones(size(t)), ...
        zeros(size(t)), ...
        sin(afa/2)*cos(omega*t), ...
        sin(afa/2)*sin(omega*t)   ]; % 姿态四元数真值