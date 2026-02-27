function res = aligni0fit(fvp, L, ts)
global glv
    fib0 = fvp.fib0k; vib0 = fvp.vib0k; pib0 = fvp.pib0k; 
    len = length(fib0);
    t = (1:len)'*ts;
    wt = glv.wie*t;
    %% fib0 fitting
    Af = [ones(len,1), cos(wt), sin(wt)];
    bf = lscov(Af, fib0);
    gf = norm(bf(1,:)+bf(2,:));  gf1 = norm(bf(1,:)+bf(3,:));
    af = bf/(gf*cos(L));
    %% vib0 fitting
    Av = [ones(len,1), wt, sin(wt), -cos(wt)];
    bv = lscov(Av, vib0);
    gv = norm(vib0(end,:))/(len*ts);
    av = bv(2:4,:)/(gv*cos(L)/glv.wie);
    %% pib0 fitting
    Ap = [ones(len,1), wt, wt.^2/2, -cos(wt), -sin(wt)];
    bp = lscov(Ap, pib0);
    gp = norm(pib0(end,:))/((len*ts)^2/2);
    ap = bp(3:5,:)/(gp*cos(L)/glv.wie^2);
    %% fitting errors
    errf = zeros(size(fib0)); errv = errf; errp = errf;
    for k=1:3
        errf(:,k) = fib0(:,k)-Af*bf(:,k);
        errv(:,k) = vib0(:,k)-Av*bv(:,k);
        errp(:,k) = pib0(:,k)-Ap*bp(:,k);
    end
    res.bf = bf; res.af = af; res.errf = errf;
    res.bv = bv; res.av = av; res.errv = errv;
    res.bp = bp; res.ap = ap; res.errp = errp; res.g = [gf;gf1;gv;gp];
    %%
    myfigure;
    subplot(311), plot(t,errf), xygo('d fib0 / m/s^2')
    subplot(312), plot(t,errv), xygo('d vib0 / m/s')
    subplot(313), plot(t,errp), xygo('d pib0 / m')

