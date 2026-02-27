function avpErr = avpcmp(avp1, avp0, type)
    if nargin<3
        type = 'avp';
    end
    t1 = avp1(:,end); avp1 = avp1(:,1:end-1);
	t0 = avp0(:,end); avp0 = avp0(:,1:end-1);
    % 确保正序
    if t1(1)>t1(2)
        t1 = flipud(t1); avp1 = flipud(avp1);
    end
    if t0(1)>t0(2)
        t0 = flipud(t0); avp0 = flipud(avp0);
    end
    % 按时间确定数据交集
    a = max(t0(1), t1(1));      
    b = min(t0(end), t1(end));  
    ai0 = find(t0>=a,1);        ai1 = find(t1>=a,1);    
    bi0 = find(t0<=b,1,'last'); bi1 = find(t1<=b,1,'last');
    avp0 = avp0(ai0:bi0, :);    avp1 = avp1(ai1:bi1, :);
    t0 = t0(ai0:bi0);           t1 = t1(ai1:bi1);
    % 在密的数据中插值
    dt0 = mean(diff(t0));       dt1 = mean(diff(t1));
    if dt1>dt0 
        t = t1;
%         avp0 = interp1(t0, avp0, t1, 'spline');
        [t, i1, i0] = intersect(t1, t0); avp0 = avp0(i0,:); avp1 = avp1(i1,:);
    else
        t = t0;
%         avp1 = interp1(t1, avp1, t0, 'spline');
        [t, i1, i0] = intersect(t1, t0); avp1 = avp1(i1,:); avp0 = avp0(i0,:); 
    end
    % 做差比较
    avpErr = [avp1-avp0, t];
    if type(1)=='a'
%         avpErr(:,1:3) = aa2phi(avp1(:,1:3), avp0(:,1:3));
        avpErr(:,1:3) = aa2mu(avp1(:,1:3), avp0(:,1:3));
    end