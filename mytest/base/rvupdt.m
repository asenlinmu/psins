function phim = rvupdt(wm)
% rotation updating using Bortz equation.
    wm = wm(:,1:3);
    rv = wm(1,:);
    for k=2:size(wm,1)
        rv1 = rv+wm(k,:)/2;
        rp = cros(rv1,wm(k,:));
        n2=rv1*rv1';
        if n2>1e-10
            n_2 = sqrt(n2)/2;
            b = 1/n2*(1-n_2*cot(n_2));
        else
            b = 1/12;
        end
        rv = rv + wm(k,:) + 1/2*rp + b*cros(rv1,rp);
    end
    phim = rv(:);

