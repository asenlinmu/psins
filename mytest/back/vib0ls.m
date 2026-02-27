function [a, b, err] = vib0ls(y, L, ts)
global glv
    [len,m] = size(y);
    wt = (1:len)'*(glv.wie*ts);
    A = [ones(len,1), wt, sin(wt), -cos(wt)];
    b = lscov(A, y);
    g = norm(y(end,:))/(len*ts);
    a = b(2:4,:)/(g*cos(L)/glv.wie);
    %%
    err = y;
    for k=1:m
        err(:,k) = y(:,k)-A*b(:,k);
    end
    figure, plot(err), grid on
